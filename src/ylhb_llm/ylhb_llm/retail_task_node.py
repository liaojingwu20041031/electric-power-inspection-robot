import copy
import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ylhb_interfaces.msg import CartState, RecognizedProduct, SayText, TaskEvent, TaskStatus

from .product_catalog import Product, ProductCatalog, product_to_dict
from .qwen_client import QwenClient, QwenClientError


TASK_IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png')


def workspace_path(*parts: str) -> str:
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_ws'))
    return os.path.join(workspace_dir, *parts)


def system_mode_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class RetailTaskNode(Node):
    def __init__(self) -> None:
        super().__init__('retail_task_node')

        self.declare_parameter('products_file', workspace_path('src', 'ylhb_llm', 'config', 'products.yaml'))
        self.declare_parameter('text_command_topic', '/retail_ai/text_command')
        self.declare_parameter('localized_objects_topic', '/perception/localized_objects')
        self.declare_parameter('task_event_topic', '/retail_ai/task_event')
        self.declare_parameter('task_status_topic', '/retail_ai/task_status')
        self.declare_parameter('say_text_topic', '/retail_ai/say_text')
        self.declare_parameter('sales_dialogue_status_topic', '/retail_ai/sales_dialogue_status')
        self.declare_parameter('cart_topic', '/retail_ai/cart')
        self.declare_parameter('start_b1_service_name', '/retail_ai/start_b1_task')
        self.declare_parameter('task_image_dir', workspace_path('src', 'ylhb_llm', 'test_images'))
        self.declare_parameter('system_mode_topic', '/retail_ai/system_mode')
        self.declare_parameter('shelf_snapshot_ttl_sec', 2.0)
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('vl_model', 'Qwen3-VL-235B-A22B-Thinking')
        self.declare_parameter('chat_model', 'qwen-max')
        self.declare_parameter('request_timeout_sec', 5.0)
        self.declare_parameter('vision_timeout_sec', 10.0)
        self.declare_parameter('publish_raw_json', True)

        products_file = self.get_parameter('products_file').value
        self.catalog = ProductCatalog.from_yaml(products_file)
        self.shelf_snapshot_ttl_sec = float(self.get_parameter('shelf_snapshot_ttl_sec').value)
        self.vl_model = self.get_parameter('vl_model').value
        self.chat_model = self.get_parameter('chat_model').value
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.vision_timeout_sec = float(self.get_parameter('vision_timeout_sec').value)
        self.publish_raw_json = bool(self.get_parameter('publish_raw_json').value)
        self.qwen = QwenClient(self.get_parameter('dashscope_base_url').value)
        self.task_image_dir = os.path.expanduser(str(self.get_parameter('task_image_dir').value))
        self.system_mode = 'ready'

        self.shelf_products: List[Tuple[Product, Dict[str, Any]]] = []
        self.latest_detected_products: List[Tuple[Product, Dict[str, Any]]] = []
        self.shelf_updated_at = 0.0
        self.latest_detected_updated_at = 0.0
        self.pending_tasks: Dict[str, Dict[str, Any]] = {}
        self.completed_task_ids = set()
        self.cart_items: Dict[str, Dict[str, Any]] = {}
        self.sales_dialogue: Dict[str, Any] = self.default_sales_dialogue()

        self.task_event_pub = self.create_publisher(
            TaskEvent, self.get_parameter('task_event_topic').value, 10)
        self.say_text_pub = self.create_publisher(
            SayText, self.get_parameter('say_text_topic').value, 10)
        self.sales_status_pub = self.create_publisher(
            String, self.get_parameter('sales_dialogue_status_topic').value, system_mode_qos())
        self.cart_pub = self.create_publisher(
            CartState, self.get_parameter('cart_topic').value, 10)

        self.create_subscription(
            String,
            self.get_parameter('text_command_topic').value,
            self.text_command_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('localized_objects_topic').value,
            self.localized_objects_callback,
            10,
        )
        self.create_subscription(
            TaskStatus,
            self.get_parameter('task_status_topic').value,
            self.task_status_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('system_mode_topic').value,
            self.system_mode_callback,
            system_mode_qos(),
        )
        start_b1_service_name = self.get_parameter('start_b1_service_name').value
        self.create_service(
            Trigger,
            start_b1_service_name,
            self.start_b1_task_callback,
        )

        self.publish_cart()
        self.publish_sales_dialogue_status()
        self.get_logger().info(
            f"Retail task node started with {len(self.catalog.products)} products. "
            f"B1 service={start_b1_service_name}, task_image_dir={self.task_image_dir}"
        )

    def system_mode_callback(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode in ('sleep', 'ready', 'mapping', 'running', 'fault'):
            self.system_mode = mode
        else:
            self.get_logger().warn(f'Ignoring unknown system_mode: {mode}')

    def start_b1_task_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        task_id = self.new_task_id('task_b_image')
        if self.system_mode in ('sleep', 'mapping', 'fault'):
            response.success = False
            response.message = json.dumps({
                'task_id': task_id,
                'stage': 'start_b1_task',
                'image_path': '',
                'say_text': '',
                'error': f'system_mode={self.system_mode}, cannot start B-1 task',
            }, ensure_ascii=False)
            return response
        if self.system_mode == 'running' and self.pending_tasks:
            response.success = False
            response.message = json.dumps({
                'task_id': task_id,
                'stage': 'start_b1_task',
                'image_path': '',
                'say_text': '',
                'error': '已有任务正在执行，不能启动新的 B-1 任务。',
            }, ensure_ascii=False)
            return response

        image_path, error = self.find_unique_task_image()
        if error:
            response.success = False
            response.message = json.dumps({
                'task_id': task_id,
                'stage': 'start_b1_task',
                'image_path': '',
                'say_text': '',
                'error': error,
            }, ensure_ascii=False)
            return response

        try:
            intent = self.qwen.analyze_image(
                image_path=image_path,
                model=self.vl_model,
                timeout_sec=self.vision_timeout_sec,
                product_names=self.catalog.names(),
            )
            description = str(intent.get('description_cn') or '我已经理解了任务书图片。')
        except QwenClientError as exc:
            response.success = False
            response.message = json.dumps({
                'task_id': task_id,
                'stage': 'image_understanding',
                'image_path': image_path,
                'say_text': '',
                'error': str(exc),
            }, ensure_ascii=False)
            self.say(task_id, f'云端图片识别超时或失败，请使用文字输入任务指令。原因：{exc}', priority=8)
            return response

        payload = {
            'schema_version': '1.0',
            'task_id': task_id,
            'timestamp': time.time(),
            'source': 'image',
            'intent': 'inspect_shelf_for_recommendation',
            'image_intent': intent,
            'selected_product': None,
            'flow': 'task_b_1',
            'next_step': 'navigate_to_shelf_then_recommend',
        }
        response.success = True

        self.say(task_id, description, priority=5)
        self.publish_task_event(task_id, 'inspect_shelf_for_recommendation', None, 'shelf', 'image',
                                float(intent.get('confidence', 0.8)), payload)
        response.message = json.dumps({
            'task_id': task_id,
            'stage': 'image_understanding',
            'image_path': image_path,
            'say_text': description,
            'error': '',
        }, ensure_ascii=False)
        return response

    def find_unique_task_image(self) -> Tuple[str, str]:
        image_dir = os.path.expanduser(self.task_image_dir)
        if not os.path.isdir(image_dir):
            return '', f'task_image_dir does not exist: {image_dir}'
        candidates = []
        for name in sorted(os.listdir(image_dir)):
            path = os.path.join(image_dir, name)
            if os.path.isfile(path) and name.lower().endswith(TASK_IMAGE_EXTENSIONS):
                candidates.append(path)
        if not candidates:
            return '', '未找到任务书图片，请在 test_images 目录保留一张 jpg/jpeg/png 图片。'
        if len(candidates) > 1:
            return '', '目录内存在多张图片，请只保留一张 jpg/jpeg/png 任务书图片。'
        return candidates[0], ''

    def text_command_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        task_id = self.new_task_id('text')
        parsed = self.parse_text_command(text)
        intent = parsed.get('intent', 'unknown')

        if intent == 'motion':
            return

        if intent == 'checkout':
            self.clear_sales_dialogue()
            self.handle_checkout(task_id)
            return

        if intent == 'cancel' and self.sales_dialogue.get('active'):
            self.clear_sales_dialogue()
            self.say(task_id, '好的，已取消本次选购。', priority=6)
            return

        if self.system_mode in ('sleep', 'mapping', 'fault'):
            self.get_logger().info(
                f'Ignoring text command while system_mode={self.system_mode}: {text}'
            )
            return
        if self.system_mode == 'running' and self.pending_tasks:
            self.get_logger().info(
                f'Ignoring text command while another task is pending: {text}'
            )
            return

        if self.sales_dialogue_expired():
            self.clear_sales_dialogue()

        if not self.qwen.available():
            self.handle_sales_fallback(task_id, text)
            return

        try:
            decision = self.qwen.parse_sales_dialogue(
                text=text,
                model=self.chat_model,
                timeout_sec=self.request_timeout_sec,
                products=self.sales_products_payload(),
                dialogue=self.sales_dialogue_payload(),
            )
        except QwenClientError as exc:
            self.get_logger().warn(f'LLM sales dialogue failed: {exc}')
            self.handle_sales_fallback(task_id, text)
            return
        self.handle_sales_decision(task_id, text, decision)

    def parse_text_command(self, text: str) -> Dict[str, Any]:
        checkout_keywords = ('多少钱', '结算', '总价', '一共', '付款')
        if any(k in text for k in checkout_keywords):
            return {'intent': 'checkout', 'confidence': 1.0}
        motion_keywords = ('前进', '后退', '左转', '右转', '停止', '停下', '刹车')
        if any(k in text for k in motion_keywords):
            return {'intent': 'motion', 'confidence': 1.0}
        cancel_keywords = ('取消', '算了', '不要了', '不买了')
        if any(k in text for k in cancel_keywords):
            return {'intent': 'cancel', 'confidence': 1.0}
        return {'intent': 'unknown', 'confidence': 0.0}

    def handle_sales_fallback(self, task_id: str, text: str) -> None:
        product = self.catalog.match_text(text)
        if product is not None:
            self.clear_sales_dialogue()
            self.execute_b2_pick(task_id, product, text, 'local_fallback', 0.8)
            return
        self.say(task_id, '云端理解失败，请直接说出商品名称，例如可乐、矿泉水或纸巾。', priority=7)

    def handle_sales_decision(self, task_id: str, text: str, decision: Dict[str, Any]) -> None:
        action = str(decision.get('action') or 'unknown').strip()
        confidence = self.safe_float(decision.get('confidence'), 0.0)
        product = self.product_from_decision(decision)
        reply = str(decision.get('reply_cn') or '').strip()

        if self.decision_has_unknown_product(decision):
            self.clear_sales_dialogue()
            self.say(task_id, '当前商品清单中没有该商品，请重新说明需要购买的商品。', priority=7)
            return

        if action == 'execute_pick':
            if product is None:
                self.say(task_id, '我还没有确定要购买的商品，请再说明一下。', priority=6)
                return
            self.clear_sales_dialogue()
            self.execute_b2_pick(task_id, product, text, 'llm_sales', confidence or 0.9)
            return

        if action == 'propose_product':
            if product is None:
                self.update_sales_dialogue(text, decision, state='asking_clarification')
                self.say(task_id, reply or '我理解了您的需求，但还不能确定商品，请再说明一下。', priority=6)
                return
            self.update_sales_dialogue(text, decision, state='awaiting_confirmation')
            self.say(task_id, reply or self.default_proposal_reply(product), priority=7)
            return

        if action == 'ask_clarification':
            self.update_sales_dialogue(text, decision, state='asking_clarification')
            self.say(task_id, reply or '请问您想购买哪一类商品？', priority=7)
            return

        if action == 'cancel':
            self.clear_sales_dialogue()
            self.say(task_id, reply or '好的，已取消本次选购。', priority=6)
            return

        self.update_sales_dialogue(text, decision, state='idle')
        self.say(task_id, reply or '我还没有理解您的需求，请重新说明要购买的商品。', priority=5)

    def execute_b2_pick(
        self,
        task_id: str,
        product: Product,
        raw_text: str,
        source: str,
        confidence: float,
    ) -> None:
        payload = {
            'schema_version': '1.0',
            'task_id': task_id,
            'timestamp': time.time(),
            'source': source,
            'intent': 'pick_item',
            'flow': 'task_b_2',
            'selected_product': product_to_dict(product),
            'target_product_id': product.id,
            'target_product_name': product.name,
            'raw_text': raw_text,
            'next_step': 'navigate_to_shelf_inspect_pick_checkout_return_start',
        }
        self.say(task_id, f'好的，为您提取{product.name}。', priority=6)
        self.publish_task_event(task_id, 'pick_item', product, 'checkout', source, confidence, payload)

    def default_sales_dialogue(self) -> Dict[str, Any]:
        return {
            'active': False,
            'state': 'idle',
            'history': [],
            'last_action': '',
            'last_product_id': '',
            'last_product_name': '',
            'need': '',
            'related_products': [],
            'rejected_product_ids': [],
            'last_reply': '',
            'expires_at': 0.0,
        }

    def sales_dialogue_expired(self) -> bool:
        expires_at = self.safe_float(self.sales_dialogue.get('expires_at'), 0.0)
        return bool(self.sales_dialogue.get('active')) and expires_at > 0.0 and time.time() > expires_at

    def clear_sales_dialogue(self) -> None:
        if self.sales_dialogue.get('last_product_id'):
            rejected = set(str(v) for v in self.sales_dialogue.get('rejected_product_ids', []))
            rejected.add(str(self.sales_dialogue.get('last_product_id')))
        else:
            rejected = set()
        self.sales_dialogue = self.default_sales_dialogue()
        self.sales_dialogue['rejected_product_ids'] = sorted(rejected)
        self.publish_sales_dialogue_status()

    def update_sales_dialogue(self, text: str, decision: Dict[str, Any], state: str) -> None:
        action = str(decision.get('action') or 'unknown')
        product = self.product_from_decision(decision)
        related = self.related_products_from_decision(decision, exclude_id=product.id if product else '')
        history = list(self.sales_dialogue.get('history') or [])
        history.append({'role': 'user', 'text': text})
        reply = str(decision.get('reply_cn') or '').strip()
        if reply:
            history.append({'role': 'assistant', 'text': reply})
        history = history[-8:]

        rejected = [str(v) for v in self.sales_dialogue.get('rejected_product_ids', []) if v]
        if action == 'propose_product' and self.sales_dialogue.get('last_product_id') and (
            product is not None and product.id != self.sales_dialogue.get('last_product_id')
        ):
            rejected.append(str(self.sales_dialogue.get('last_product_id')))

        self.sales_dialogue = {
            'active': state in ('awaiting_confirmation', 'asking_clarification'),
            'state': state,
            'history': history,
            'last_action': action,
            'last_product_id': product.id if product is not None else '',
            'last_product_name': product.name if product is not None else '',
            'need': str(decision.get('need') or self.sales_dialogue.get('need') or ''),
            'related_products': related,
            'rejected_product_ids': sorted(set(rejected)),
            'last_reply': reply,
            'expires_at': time.time() + 120.0 if state in ('awaiting_confirmation', 'asking_clarification') else 0.0,
        }
        self.publish_sales_dialogue_status()

    def publish_sales_dialogue_status(self) -> None:
        product = self.catalog.get(str(self.sales_dialogue.get('last_product_id') or ''))
        payload = {
            'active': bool(self.sales_dialogue.get('active')),
            'state': str(self.sales_dialogue.get('state') or 'idle'),
            'last_action': str(self.sales_dialogue.get('last_action') or ''),
            'need': str(self.sales_dialogue.get('need') or ''),
            'primary_product_id': product.id if product is not None else '',
            'primary_product_name': product.name if product is not None else '',
            'primary_price': product.price if product is not None else 0.0,
            'related_products': self.sales_dialogue.get('related_products') or [],
            'last_reply': str(self.sales_dialogue.get('last_reply') or ''),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.sales_status_pub.publish(msg)

    def sales_dialogue_payload(self) -> Dict[str, Any]:
        return {
            'active': bool(self.sales_dialogue.get('active')),
            'state': str(self.sales_dialogue.get('state') or 'idle'),
            'conversation_history': list(self.sales_dialogue.get('history') or []),
            'last_action': str(self.sales_dialogue.get('last_action') or ''),
            'last_product_id': str(self.sales_dialogue.get('last_product_id') or ''),
            'last_product_name': str(self.sales_dialogue.get('last_product_name') or ''),
            'need': str(self.sales_dialogue.get('need') or ''),
            'related_products': list(self.sales_dialogue.get('related_products') or []),
            'rejected_product_ids': list(self.sales_dialogue.get('rejected_product_ids') or []),
        }

    def sales_products_payload(self) -> List[Dict[str, Any]]:
        products = []
        for product in self.catalog.products:
            products.append({
                'id': product.id,
                'name': product.name,
                'category': product.category,
                'price': product.price,
                'aliases': list(product.aliases),
                'priority_for_intents': dict(product.priority_for_intents),
                'selling_points': list(product.selling_points),
                'suitable_needs': list(product.suitable_needs),
            })
        return products

    def product_from_decision(self, decision: Dict[str, Any]) -> Optional[Product]:
        product_id = str(
            decision.get('primary_product_id') or decision.get('product_id') or ''
        ).strip()
        if product_id:
            return self.catalog.get(product_id)
        product_name = str(
            decision.get('primary_product_name') or decision.get('product_name') or ''
        ).strip()
        return self.catalog.match_text(product_name) if product_name else None

    def decision_has_unknown_product(self, decision: Dict[str, Any]) -> bool:
        product_id = str(decision.get('primary_product_id') or decision.get('product_id') or '').strip()
        if product_id and self.catalog.get(product_id) is None:
            return True
        for item in decision.get('related_products') or []:
            if not isinstance(item, dict):
                continue
            related_id = str(item.get('product_id') or '').strip()
            if related_id and self.catalog.get(related_id) is None:
                return True
        return False

    def related_products_from_decision(self, decision: Dict[str, Any], exclude_id: str = '') -> List[Dict[str, Any]]:
        related: List[Dict[str, Any]] = []
        seen = set([exclude_id] if exclude_id else [])
        for item in decision.get('related_products') or []:
            if not isinstance(item, dict):
                continue
            product_id = str(item.get('product_id') or '').strip()
            product = self.catalog.get(product_id) if product_id else None
            if product is None:
                product_name = str(item.get('product_name') or '').strip()
                product = self.catalog.match_text(product_name) if product_name else None
            if product is None or product.id in seen:
                continue
            seen.add(product.id)
            related.append({
                'product_id': product.id,
                'product_name': product.name,
                'price': product.price,
                'reason_cn': str(item.get('reason_cn') or ''),
            })
            if len(related) >= 2:
                break
        return related

    def default_proposal_reply(self, product: Product) -> str:
        return f'我主推{product.name}，价格{self.format_price(product.price)}元。确认购买请说确认，想换商品请说换一个。'

    def safe_float(self, value: Any, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def localized_objects_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid localized objects JSON: {exc}')
            return

        matches: List[Tuple[Product, Dict[str, Any]]] = []
        for obj in payload.get('objects', []):
            text = ' '.join(str(obj.get(k, '')) for k in ('class_name', 'label', 'name', 'item_name'))
            product = self.catalog.match_text(text)
            if product is not None:
                matches.append((product, obj))
        self.latest_detected_products = matches
        self.latest_detected_updated_at = time.monotonic()
        if matches:
            self.shelf_products = self.dedupe_shelf(matches)
            self.shelf_updated_at = time.monotonic()

    def task_status_callback(self, msg: TaskStatus) -> None:
        task_id = msg.task_id
        status = msg.status
        context = self.pending_tasks.get(task_id)
        product = context.get('product') if context else None
        if status == 'started':
            if not context or context.get('workflow') not in ('task_a_motion',):
                self.say(task_id, '已开始执行任务。', priority=3)
        elif status == 'succeeded':
            if context and context.get('workflow') == 'task_b_1_recommend':
                self.handle_shelf_inspection_succeeded(task_id, context)
            elif context and context.get('workflow') == 'task_c_checkout':
                self.handle_checkout_status_succeeded(task_id, msg.stage, context)
            elif product is not None:
                self.add_to_cart_once(task_id, product)
                self.publish_cart()
                self.say(task_id, f'{product.name}已放入结算区。', priority=6)
                self.completed_task_ids.add(task_id)
                self.pending_tasks.pop(task_id, None)
        elif status in ('failed', 'rejected'):
            reason = msg.reason or '任务执行失败'
            self.say(task_id, reason, priority=8)
            self.pending_tasks.pop(task_id, None)

    def handle_checkout(self, task_id: str) -> None:
        payload = {
            'schema_version': '1.0',
            'task_id': task_id,
            'timestamp': time.time(),
            'source': 'checkout',
            'intent': 'checkout',
            'flow': 'task_c',
            'next_step': 'navigate_to_checkout_and_recognize_items',
        }
        self.publish_task_event(task_id, 'checkout', None, 'checkout', 'checkout', 1.0, payload)

    def choose_product_from_shelf(self, intent: Dict[str, Any]) -> Tuple[Optional[Product], str]:
        if time.monotonic() - self.shelf_updated_at > self.shelf_snapshot_ttl_sec:
            return None, 'shelf snapshot expired'
        need = str(intent.get('need') or '')
        preferred = [str(v) for v in intent.get('preferred_categories', []) if v]
        best: Optional[Product] = None
        best_score = -1.0
        for product, _obj in self.shelf_products:
            score = self.catalog.score_for_need(product, need, preferred)
            if score > best_score:
                best = product
                best_score = score
        if best is None or best_score <= 0.0:
            return None, 'no matching shelf product'
        return best, f'score={best_score:.1f}'

    def publish_task_event(
        self,
        task_id: str,
        intent: str,
        product: Optional[Product],
        destination: str,
        source: str,
        confidence: float,
        raw: Dict[str, Any],
    ) -> None:
        msg = TaskEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.intent = intent
        msg.item_id = product.id if product is not None else ''
        msg.item_name = product.name if product is not None else ''
        msg.destination = destination
        msg.confidence = float(confidence)
        msg.source = source
        msg.requires_ack = True
        msg.raw_json = json.dumps(raw, ensure_ascii=False) if self.publish_raw_json else ''
        context = self.pending_tasks.get(task_id, {})
        workflow = raw.get('flow') or context.get('workflow')
        if intent == 'inspect_shelf_for_recommendation':
            workflow = 'task_b_1_recommend'
        elif intent == 'checkout':
            workflow = 'task_c_checkout'
        elif intent == 'return_start':
            workflow = 'task_c_checkout'
        context.update({
            'workflow': workflow,
            'intent': intent,
            'product': product,
            'destination': destination,
            'source': source,
            'raw': copy.deepcopy(raw),
        })
        self.pending_tasks[task_id] = context
        self.task_event_pub.publish(msg)

    def handle_shelf_inspection_succeeded(self, task_id: str, context: Dict[str, Any]) -> None:
        intent = dict(context.get('raw', {}).get('image_intent') or {})
        product, reason = self.choose_product_from_shelf(intent)
        if product is None:
            self.say(task_id, '当前货架识别结果已过期或没有匹配商品，请重新对准货架。', priority=7)
            return
        payload = dict(context.get('raw') or {})
        payload.update({
            'intent': 'pick_item',
            'selected_product': product_to_dict(product),
            'reason': reason,
            'next_step': 'pick_recommended_item_and_place_to_checkout',
        })
        self.say(task_id, f'为您推荐选购商品为：{product.name}。', priority=6)
        self.publish_task_event(task_id, 'pick_item', product, 'checkout', 'image',
                                float(intent.get('confidence', 0.8)), payload)

    def handle_checkout_status_succeeded(
        self,
        task_id: str,
        stage: str,
        context: Dict[str, Any],
    ) -> None:
        stage = stage or ''
        if stage in ('checkout_inspect', 'inspect_checkout', 'recognize_checkout', 'navigate_to_checkout', 'checkout'):
            items = self.checkout_items_from_latest_detection()
            context['checkout_items'] = items
            self.publish_cart_from_items(items)
            if not items:
                self.say(task_id, '结算区内没有识别到商品。', priority=8)
            else:
                self.say(task_id, f"您选购的商品有：{self.format_item_list(items)}。", priority=8)
            payload = dict(context.get('raw') or {})
            payload.update({
                'intent': 'return_start',
                'recognized_items': [self.cart_item_to_raw(v) for v in items.values()],
                'next_step': 'return_to_start_then_say_total',
            })
            self.publish_task_event(task_id, 'return_start', None, 'start', 'checkout', 1.0, payload)
            return

        if stage in ('return_start', 'return_to_start', 'start', 'arrive_start'):
            items = context.get('checkout_items') or {}
            total = self.total_for_items(items)
            self.say(task_id, f'您购买的商品总价为{self.format_price(total)}元。', priority=8)
            self.completed_task_ids.add(task_id)
            self.pending_tasks.pop(task_id, None)

    def checkout_items_from_latest_detection(self) -> Dict[str, Dict[str, Any]]:
        if time.monotonic() - self.latest_detected_updated_at > self.shelf_snapshot_ttl_sec:
            return {}
        items: Dict[str, Dict[str, Any]] = {}
        for product, obj in self.latest_detected_products:
            if product.id not in items:
                items[product.id] = {
                    'item_id': product.id,
                    'name': product.name,
                    'category': product.category,
                    'quantity': 0,
                    'unit_price': product.price,
                    'source_task_ids': [],
                    'detections': [],
                }
            items[product.id]['quantity'] += 1
            items[product.id]['detections'].append(obj)
        return items

    def publish_cart_from_items(self, items: Dict[str, Dict[str, Any]]) -> None:
        self.cart_items = {
            item_id: {
                'item_id': item['item_id'],
                'name': item['name'],
                'category': item['category'],
                'quantity': item['quantity'],
                'unit_price': item['unit_price'],
                'source_task_ids': item.get('source_task_ids', []),
            }
            for item_id, item in items.items()
        }
        self.publish_cart()

    def format_item_list(self, items: Dict[str, Dict[str, Any]]) -> str:
        return '、'.join(f"{int(item['quantity'])}件{item['name']}" for item in items.values())

    def total_for_items(self, items: Dict[str, Dict[str, Any]]) -> float:
        return sum(float(item['quantity']) * float(item['unit_price']) for item in items.values())

    def format_price(self, value: float) -> str:
        return str(int(value)) if float(value).is_integer() else str(value)

    def cart_item_to_raw(self, item: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'item_id': item['item_id'],
            'name': item['name'],
            'category': item['category'],
            'quantity': item['quantity'],
            'unit_price': item['unit_price'],
        }

    def say(self, task_id: str, text: str, priority: int = 5, interrupt: bool = False) -> None:
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.priority = int(priority)
        msg.interrupt = bool(interrupt)
        msg.text = text
        self.say_text_pub.publish(msg)

    def add_to_cart_once(self, task_id: str, product: Product) -> None:
        if task_id in self.completed_task_ids:
            return
        if product.id not in self.cart_items:
            self.cart_items[product.id] = {
                'item_id': product.id,
                'name': product.name,
                'category': product.category,
                'quantity': 0,
                'unit_price': product.price,
                'source_task_ids': [],
            }
        item = self.cart_items[product.id]
        if task_id not in item['source_task_ids']:
            item['quantity'] += 1
            item['source_task_ids'].append(task_id)

    def publish_cart(self) -> None:
        msg = CartState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.schema_version = '1.0'
        total = 0.0
        raw_items = []
        for item in self.cart_items.values():
            product_msg = RecognizedProduct()
            product_msg.item_id = item['item_id']
            product_msg.name = item['name']
            product_msg.category = item['category']
            product_msg.quantity = int(item['quantity'])
            product_msg.unit_price = float(item['unit_price'])
            product_msg.confidence = 1.0
            product_msg.source_task_id = ','.join(item['source_task_ids'])
            product_msg.source = 'task_status'
            product_msg.raw_json = ''
            msg.items.append(product_msg)
            total += product_msg.quantity * product_msg.unit_price
            raw_items.append(item)
        msg.total = float(total)
        msg.raw_json = json.dumps({'items': raw_items, 'total': total}, ensure_ascii=False)
        self.cart_pub.publish(msg)

    def dedupe_shelf(self, matches: List[Tuple[Product, Dict[str, Any]]]) -> List[Tuple[Product, Dict[str, Any]]]:
        deduped: Dict[str, Tuple[Product, Dict[str, Any]]] = {}
        for product, obj in matches:
            prev = deduped.get(product.id)
            if prev is None:
                deduped[product.id] = (product, obj)
                continue
            prev_conf = float(prev[1].get('confidence', 0.0))
            cur_conf = float(obj.get('confidence', 0.0))
            if cur_conf > prev_conf:
                deduped[product.id] = (product, obj)
        return list(deduped.values())

    def new_task_id(self, prefix: str) -> str:
        return f'{prefix}_{int(time.time() * 1000)}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RetailTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
