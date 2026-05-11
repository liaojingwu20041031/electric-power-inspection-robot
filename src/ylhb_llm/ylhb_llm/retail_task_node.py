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
CONFIRM_WORDS = ('确认', '确定', '就这个', '我要这个', '开始取货', '帮我拿这个')
WEAK_CONFIRM_WORDS = ('是', '对', '好', '可以', '嗯')
NEGATIVE_WORDS = ('不需要', '不用', '不要', '算了')
MODIFY_WORDS = ('换一个', '不对', '不要这个', '重新推荐')
WAIT_CONFIRM_PRODUCT = 'confirm_product'
WAIT_CHOOSE_ALTERNATIVE = 'choose_alternative'
WAIT_ASK_ADDON = 'ask_addon'
WAIT_CLARIFY_PRODUCT = 'clarify_product'


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
        self.declare_parameter('fast_classifier_model', 'qwen3.6-flash')
        self.declare_parameter('classifier_timeout_sec', 4.0)
        self.declare_parameter('enable_fast_sales_classifier', True)
        self.declare_parameter('classifier_confidence_threshold', 0.72)
        self.declare_parameter('enable_plus_fallback', True)
        self.declare_parameter('plus_fallback_confidence_threshold', 0.62)
        self.declare_parameter('sales_reply_min_chars', 90)
        self.declare_parameter('sales_reply_max_chars', 180)
        self.declare_parameter('voice_requires_confirmation', True)

        products_file = self.get_parameter('products_file').value
        self.catalog = ProductCatalog.from_yaml(products_file)
        self.shelf_snapshot_ttl_sec = float(self.get_parameter('shelf_snapshot_ttl_sec').value)
        self.vl_model = self.get_parameter('vl_model').value
        self.chat_model = self.get_parameter('chat_model').value
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.vision_timeout_sec = float(self.get_parameter('vision_timeout_sec').value)
        self.publish_raw_json = bool(self.get_parameter('publish_raw_json').value)
        self.fast_classifier_model = str(self.get_parameter('fast_classifier_model').value)
        self.classifier_timeout_sec = float(self.get_parameter('classifier_timeout_sec').value)
        self.enable_fast_sales_classifier = bool(self.get_parameter('enable_fast_sales_classifier').value)
        self.classifier_confidence_threshold = float(self.get_parameter('classifier_confidence_threshold').value)
        self.enable_plus_fallback = bool(self.get_parameter('enable_plus_fallback').value)
        self.plus_fallback_confidence_threshold = float(self.get_parameter('plus_fallback_confidence_threshold').value)
        self.sales_reply_min_chars = int(self.get_parameter('sales_reply_min_chars').value)
        self.sales_reply_max_chars = int(self.get_parameter('sales_reply_max_chars').value)
        self.voice_requires_confirmation = bool(self.get_parameter('voice_requires_confirmation').value)
        self.qwen = QwenClient(self.get_parameter('dashscope_base_url').value)
        self.task_image_dir = os.path.expanduser(str(self.get_parameter('task_image_dir').value))
        self.system_mode = 'ready'

        self.shelf_products: List[Tuple[Product, Dict[str, Any]]] = []
        self.latest_detected_products: List[Tuple[Product, Dict[str, Any]]] = []
        self.shelf_updated_at = 0.0
        self.latest_detected_updated_at = 0.0
        self.pending_tasks: Dict[str, Dict[str, Any]] = {}
        self.completed_task_ids = set()
        self.executed_task_request_ids = set()
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
        text, command_meta = self.parse_text_command_message(msg.data)
        if not text:
            return
        task_request_id = str(command_meta.get('task_request_id') or '').strip()
        task_id = task_request_id or self.new_task_id('text')
        parsed = self.parse_text_command(text)
        intent = parsed.get('intent', 'unknown')
        source = str(command_meta.get('source') or 'text')
        route = str(command_meta.get('route') or '')

        if route in ('voice_close', 'global_safety'):
            return

        if route == 'system_feedback':
            self.say(
                task_id,
                '我已经收到您的语音，但当前扬声器可能没有成功播放。请检查音频输出设备是否被占用，或切换到默认输出设备。',
                priority=8,
            )
            return

        if intent == 'motion':
            return

        if intent == 'checkout':
            self.clear_sales_dialogue()
            self.handle_checkout(task_id)
            return

        if intent == 'cancel' and self.sales_dialogue.get('active'):
            self.handle_negative_or_cancel(task_id, text)
            return
        if route == 'global_cancel':
            self.clear_sales_dialogue()
            return

        if self.is_status_query_text(text):
            self.say(task_id, self.status_query_reply(), priority=7)
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

        if self.is_confirm_text(text):
            self.handle_confirm(task_id, text, command_meta)
            return

        if self.is_weak_confirm_text(text) and self.sales_dialogue.get('active'):
            self.handle_weak_confirm(task_id)
            return

        if self.is_negative_text(text):
            self.handle_negative_or_cancel(task_id, text)
            return

        if self.is_modify_text(text):
            decision = self.modify_current_proposal()
            if decision is not None:
                self.handle_sales_decision(task_id, text, decision, command_meta)
            else:
                self.say(task_id, '您可以重新说明想要的商品类型，我再为您推荐。', priority=6)
            return

        general_reply = self.try_local_general_qa(text, command_meta)
        if general_reply:
            self.say(task_id, general_reply, priority=7)
            return

        fast_decision = self.try_local_fast_sales_decision(text, command_meta)
        if fast_decision is not None:
            self.handle_sales_decision(task_id, text, fast_decision, command_meta)
            return

        if self.enable_fast_sales_classifier and self.qwen.available():
            try:
                category = self.qwen.classify_sales_category(
                    text=text,
                    model=self.fast_classifier_model,
                    timeout_sec=self.classifier_timeout_sec,
                    dialogue=self.sales_dialogue_payload(current_source=source),
                )
                decision = self.decision_from_category(text, category, command_meta)
                if decision is not None:
                    self.handle_sales_decision(task_id, text, decision, command_meta)
                    return
                conf = self.safe_float(category.get('confidence'), 0.0)
                if not self.enable_plus_fallback or conf >= self.plus_fallback_confidence_threshold:
                    self.say(task_id, self.reply_for_unclear_category(category), priority=6)
                    return
            except QwenClientError as exc:
                self.get_logger().warn(f'Flash sales classifier failed: {exc}')

        if not self.qwen.available():
            self.handle_sales_fallback(task_id, text, command_meta)
            return

        try:
            decision = self.qwen.parse_sales_dialogue(
                text=text,
                model=self.chat_model,
                timeout_sec=self.request_timeout_sec,
                products=self.sales_products_payload(),
                dialogue=self.sales_dialogue_payload(current_source=source),
            )
        except QwenClientError as exc:
            self.get_logger().warn(f'LLM sales dialogue failed: {exc}')
            self.handle_sales_fallback(task_id, text, command_meta)
            return
        self.handle_sales_decision(task_id, text, decision, command_meta)

    def parse_text_command_message(self, data: str) -> Tuple[str, Dict[str, Any]]:
        raw = data.strip()
        if not raw.startswith('{'):
            return raw, {}
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return raw, {}
        if not isinstance(payload, dict):
            return raw, {}
        text = str(payload.get('text') or '').strip()
        return text, payload

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

    def handle_sales_fallback(self, task_id: str, text: str, command_meta: Optional[Dict[str, Any]] = None) -> None:
        command_meta = command_meta or {}
        product = self.catalog.match_text(text)
        if product is not None:
            if command_meta.get('source') == 'voice':
                decision = {
                    'action': 'propose_product',
                    'primary_product_id': product.id,
                    'primary_product_name': product.name,
                    'related_products': [],
                    'reply_cn': self.default_proposal_reply(product),
                    'confidence': 0.75,
                    'requires_confirmation': True,
                    'reason_cn': '本地商品名兜底匹配，需要用户确认后再执行。',
                }
                self.update_sales_dialogue(text, decision, state='awaiting_confirmation')
                self.say(task_id, decision['reply_cn'], priority=7)
                return
            self.clear_sales_dialogue()
            self.execute_b2_pick(task_id, product, text, 'local_fallback', 0.8, command_meta)
            return
        self.say(task_id, '云端理解失败，请直接说出商品名称，例如可乐、矿泉水或纸巾。', priority=7)

    def handle_confirm(self, task_id: str, text: str, command_meta: Dict[str, Any]) -> None:
        product_id = str(
            self.sales_dialogue.get('pending_product_id')
            or self.sales_dialogue.get('last_product_id')
            or ''
        )
        product = self.catalog.get(product_id)
        waiting_for = str(self.sales_dialogue.get('waiting_for') or '')
        if product is None or self.sales_dialogue.get('state') != 'awaiting_confirmation':
            self.say(task_id, '当前没有待确认商品，请先说出您的需求。', priority=6)
            return
        if waiting_for and waiting_for != WAIT_CONFIRM_PRODUCT:
            self.say(task_id, '请先完成当前问题，或者说取消。', priority=6)
            return
        self.clear_sales_dialogue()
        self.execute_b2_pick(
            task_id,
            product,
            text,
            'voice_confirm' if command_meta.get('source') == 'voice' else 'text_confirm',
            self.safe_float(command_meta.get('confidence'), 0.95),
            command_meta,
        )

    def handle_negative_or_cancel(self, task_id: str, text: str) -> None:
        waiting_for = str(self.sales_dialogue.get('waiting_for') or '')
        product = self.catalog.get(str(self.sales_dialogue.get('pending_product_id') or self.sales_dialogue.get('last_product_id') or ''))
        if waiting_for == WAIT_ASK_ADDON and product is not None:
            decision = self.build_propose_decision(
                product=product,
                alternatives=self.products_from_related(self.sales_dialogue.get('related_products') or []),
                need=str(self.sales_dialogue.get('need') or 'unknown'),
                reason='用户不需要搭配，保留当前主商品。',
                reply=f'好的，那只为您保留{product.name}。请说“确认”开始取货，或说“换一个”重新选择。',
                confidence=0.95,
            )
            self.update_sales_dialogue(text, decision, state='awaiting_confirmation', waiting_for=WAIT_CONFIRM_PRODUCT)
            self.say(task_id, decision['reply_cn'], priority=6)
            return
        if self.sales_dialogue.get('active'):
            self.clear_sales_dialogue()
            self.say(task_id, '好的，已取消当前推荐。您可以重新说出需求。', priority=6)
            return
        self.say(task_id, '好的，您可以重新说明需要什么。', priority=5)

    def handle_weak_confirm(self, task_id: str) -> None:
        waiting_for = str(self.sales_dialogue.get('waiting_for') or '')
        product = self.catalog.get(str(self.sales_dialogue.get('pending_product_id') or self.sales_dialogue.get('last_product_id') or ''))
        if waiting_for == WAIT_ASK_ADDON and product is not None:
            self.say(task_id, f'可以，我可以继续推荐搭配；主商品仍保留{product.name}。如果要开始取货，请说“确认”。', priority=6)
            return
        if waiting_for == WAIT_CONFIRM_PRODUCT and product is not None:
            self.say(task_id, f'如果需要{product.name}，请说“确认”开始取货。', priority=6)
            return
        self.say(task_id, '请直接说商品名，或者说“确认”开始取货。', priority=5)

    def try_local_fast_sales_decision(
        self,
        text: str,
        command_meta: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        source = str(command_meta.get('source') or 'text')
        normalized = self.normalize_cn(text)
        product = self.catalog.match_text(normalized)
        if product is not None:
            if source == 'voice' and self.voice_requires_confirmation:
                return self.build_propose_decision(
                    product=product,
                    alternatives=[],
                    need='explicit_product',
                    reason='本地商品别名命中。',
                    reply=self.build_sales_reply(
                        need_text='已经明确说出了商品',
                        product=product,
                        alternatives=[],
                        reason=f'{product.name}是当前可选商品，可以为您提取',
                    ),
                    confidence=0.96,
                )
            return self.build_execute_decision(product, confidence=0.96)

        if any(k in normalized for k in ('口渴', '喝水', '渴了', '想喝水', '解渴')):
            return self.decision_by_need_category('thirsty', normalized)
        if any(k in normalized for k in ('饿', '吃东西', '零食', '解馋')):
            return self.decision_by_need_category('hungry', normalized)
        if any(k in normalized for k in ('困了', '没精神', '提神')):
            return self.decision_by_need_category('sleepy', normalized)
        if any(k in normalized for k in ('生活用品', '日用品')):
            return self.decision_by_need_category('daily_goods', normalized)
        if any(k in normalized for k in ('擦嘴', '纸巾', '抽纸', '擦东西', '弄脏', '清洁')):
            return self.decision_by_need_category('tissue', normalized)
        return None

    def try_local_general_qa(self, text: str, command_meta: Dict[str, Any]) -> str:
        del command_meta
        normalized = self.normalize_cn(text)
        if any(k in normalized for k in ('介绍一下自己', '你是谁', '自我介绍', '介绍自己')):
            return (
                '我是智慧零售机器人小零，可以通过语音理解您的购物需求，'
                '并完成商品推荐、确认取货和结算区送达。'
                '您可以说“我口渴了”“我有点饿”或者“我要纸巾”，'
                '我会先推荐合适商品，等您说“确认”后再开始取货。'
            )
        if any(k in normalized for k in ('你能做什么', '你会做什么', '有什么功能', '能干什么')):
            return (
                '我能做三件事：听懂您的语音或文字购物需求，推荐合适商品，'
                '并在您确认后去货架取货、送到结算区。'
                '您还可以说“换一个”“不要碳酸的”“便宜点”，我会继续调整推荐。'
            )
        if any(k in normalized for k in ('怎么用', '怎么和你说', '使用方法', '语音怎么用')):
            return (
                '使用时先说“小零小零”唤醒我，然后直接说明需求。'
                '比如说“我口渴了”，我会推荐饮品；说“我有点饿”，我会推荐零食。'
                '我只会在您明确说“确认”后开始取货，避免误操作。'
            )
        if any(k in normalized for k in ('有什么商品', '卖什么', '有哪些东西', '商品列表')):
            return self.catalog_intro_reply()
        if self.is_status_query_text(text):
            return self.status_query_reply()
        return ''

    def is_status_query_text(self, text: str) -> bool:
        normalized = self.normalize_cn(text)
        return any(k in normalized for k in ('现在状态', '当前状态', '任务状态'))

    def status_query_reply(self) -> str:
        if self.system_mode == 'running':
            return '当前机器人正在执行任务，请等待完成，或者说“取消任务”。'
        if self.sales_dialogue.get('active'):
            product_name = str(self.sales_dialogue.get('last_product_name') or '')
            if product_name:
                return f'当前正在等待您确认{product_name}。需要的话请说“确认”，想换商品请说“换一个”。'
        return '当前机器人处于待命状态，您可以直接说出想买的商品或需求。'

    def decision_from_category(
        self,
        text: str,
        category: Dict[str, Any],
        command_meta: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        del command_meta
        action = str(category.get('dialogue_action') or 'unknown')
        need = str(category.get('need_category') or 'unknown')
        product_mention = str(category.get('product_mention') or '').strip()
        confidence = self.safe_float(category.get('confidence'), 0.0)
        if confidence < self.classifier_confidence_threshold:
            return None

        if action == 'ask_catalog':
            return {
                'task': 'b2_sales',
                'action': 'ask_clarification',
                'primary_product_id': '',
                'primary_product_name': '',
                'related_products': [],
                'reply_cn': self.catalog_intro_reply(),
                'confidence': confidence,
                'requires_confirmation': False,
                'reason_cn': '用户询问商品范围。',
                'waiting_for': WAIT_CLARIFY_PRODUCT,
            }
        if action == 'cancel':
            return {
                'task': 'b2_sales',
                'action': 'cancel',
                'reply_cn': '好的，已取消当前推荐。您可以重新说出需求。',
                'confidence': confidence,
                'requires_confirmation': False,
                'reason_cn': '用户取消选购。',
            }
        if action == 'modify':
            return self.modify_current_proposal_by_category(category)
        if product_mention:
            product = self.catalog.match_text(product_mention)
            if product is not None:
                return self.build_propose_decision(
                    product=product,
                    alternatives=[],
                    need=need,
                    reason=str(category.get('reason_cn') or '用户明确提到商品。'),
                    reply=self.build_sales_reply(
                        need_text='已经明确说出了商品',
                        product=product,
                        alternatives=[],
                        reason=f'{product.name}符合您的当前需求',
                    ),
                    confidence=confidence,
                )
            return {
                'task': 'b2_sales',
                'action': 'ask_clarification',
                'reply_cn': f'我不太确定您说的“{product_mention}”是哪件商品，请重新说商品名。',
                'confidence': confidence,
                'requires_confirmation': False,
                'reason_cn': '商品提及未命中商品库。',
                'waiting_for': WAIT_CLARIFY_PRODUCT,
            }
        if action == 'recommend':
            return self.decision_by_need_category(
                need,
                text,
                positive_constraints=category.get('positive_constraints') or [],
                negative_constraints=category.get('negative_constraints') or [],
                confidence=confidence,
            )
        return None

    def decision_by_need_category(
        self,
        need_category: str,
        text: str,
        positive_constraints: Optional[List[str]] = None,
        negative_constraints: Optional[List[str]] = None,
        confidence: float = 0.9,
    ) -> Optional[Dict[str, Any]]:
        del text
        positive_constraints = positive_constraints or []
        negative_constraints = negative_constraints or []
        candidates = self.rank_products_for_category(need_category, positive_constraints, negative_constraints)
        if not candidates:
            return None
        primary = candidates[0]
        alternatives = candidates[1:3]
        reply = self.build_sales_reply(
            need_text=self.need_text_cn(need_category),
            product=primary,
            alternatives=alternatives,
            reason=self.product_reason_for_need(primary, need_category, positive_constraints, negative_constraints),
        )
        return self.build_propose_decision(
            product=primary,
            alternatives=alternatives,
            need=need_category,
            reason=f'按类别 {need_category} 和约束快速推荐。',
            reply=reply,
            confidence=confidence,
        )

    def rank_products_for_category(
        self,
        need_category: str,
        positive_constraints: List[str],
        negative_constraints: List[str],
    ) -> List[Product]:
        scored: List[Tuple[float, Product]] = []
        rejected = set(str(v) for v in self.sales_dialogue.get('rejected_product_ids', []) if v)
        for product in self.catalog.products:
            score = float(product.priority_for_intents.get(need_category, 0.0))
            text_blob = ' '.join(
                [product.name, product.category]
                + list(product.aliases)
                + list(product.selling_points)
                + list(product.suitable_needs)
            )

            if need_category == 'daily_goods' and product.category in ('tissue', 'hygiene'):
                score += 70
            if need_category == 'thirsty' and product.category in ('water', 'milk', 'soda'):
                score += 40
            if need_category == 'hungry' and product.category in ('snack', 'fruit', 'milk_drink'):
                score += 40
            if need_category == 'snack' and product.category == 'snack':
                score += 40
            if need_category == 'drink' and product.category in ('water', 'milk', 'soda', 'milk_drink', 'energy_drink', 'coffee'):
                score += 30
            if need_category in ('sleepy', 'energy') and product.category in ('coffee', 'energy_drink'):
                score += 40
            if need_category in ('tissue', 'clean') and product.category == 'tissue':
                score += 60

            if 'cheap' in positive_constraints and product.price <= 3:
                score += 20
            if 'non_carbonated' in positive_constraints and product.category not in ('soda', 'energy_drink'):
                score += 20
            if 'healthy' in positive_constraints and product.category in ('water', 'fruit', 'milk'):
                score += 20
            if 'sweet' in positive_constraints and any(k in text_blob for k in ('甜', '奶', '饼干')):
                score += 15
            if 'salty' in positive_constraints and any(k in text_blob for k in ('咸', '薯片', '瓜子')):
                score += 15
            if 'filling' in positive_constraints and any(k in text_blob for k in ('饱', '饼干', '营养')):
                score += 15

            if 'carbonated' in negative_constraints and product.category == 'soda':
                score -= 100
            if 'expensive' in negative_constraints and product.price >= 6:
                score -= 20
            if 'sweet' in negative_constraints and any(k in text_blob for k in ('甜', '奶', '饼干')):
                score -= 20
            if product.id in rejected:
                score -= 50

            if score > 0:
                scored.append((score, product))
        scored.sort(key=lambda item: (-item[0], item[1].price))
        return [product for _score, product in scored]

    def build_sales_reply(
        self,
        need_text: str,
        product: Product,
        alternatives: List[Product],
        reason: str,
    ) -> str:
        alt_text = ''
        if alternatives:
            alt_names = '、'.join(p.name for p in alternatives[:2])
            alt_text = f'如果想换一种，也可以考虑{alt_names}。'
        if need_text in ('已经明确说出了商品', 'explicit_product'):
            prefix = f'好的，我已经帮您选中{product.name}。'
        else:
            prefix = f'我理解您现在{need_text}。'
        reply = (
            f'{prefix}'
            f'我主推{product.name}，因为{reason}。'
            f'{alt_text}'
            f'需要{product.name}的话，请说“确认”；想换商品请说“换一个”。'
        )
        return self.polish_sales_reply_length(reply)

    def polish_sales_reply_length(self, reply: str) -> str:
        reply = reply.replace('。 ', '。').replace('； ', '；')
        reply = reply.replace('  ', ' ')
        if len(reply) > self.sales_reply_max_chars:
            replacements = (
                ('比较适合', '适合'),
                ('如果您想要', '如果想'),
                ('我会先为您保留这个推荐，确认后才会开始取货。', ''),
                ('当前可选商品范围内', '可选范围内'),
            )
            for old, new in replacements:
                reply = reply.replace(old, new)
        if len(reply) > self.sales_reply_max_chars:
            confirm = ' 请说“确认”开始取货，或说“换一个”。'
            reply = reply[: max(0, self.sales_reply_max_chars - len(confirm))].rstrip('，。；; ')
            reply += confirm
        return reply

    def build_propose_decision(
        self,
        product: Product,
        alternatives: List[Product],
        need: str,
        reason: str,
        reply: str,
        confidence: float,
    ) -> Dict[str, Any]:
        return {
            'task': 'b2_sales',
            'action': 'propose_product',
            'need': need,
            'primary_product_id': product.id,
            'primary_product_name': product.name,
            'related_products': [
                {
                    'product_id': item.id,
                    'product_name': item.name,
                    'reason_cn': self.product_reason_for_need(item, need, [], []),
                }
                for item in alternatives[:2]
            ],
            'reply_cn': reply,
            'confidence': confidence,
            'requires_confirmation': True,
            'reason_cn': reason,
            'waiting_for': WAIT_CONFIRM_PRODUCT,
        }

    def build_execute_decision(self, product: Product, confidence: float) -> Dict[str, Any]:
        return {
            'task': 'b2_sales',
            'action': 'execute_pick',
            'primary_product_id': product.id,
            'primary_product_name': product.name,
            'related_products': [],
            'reply_cn': f'好的，为您提取{product.name}。',
            'confidence': confidence,
            'requires_confirmation': False,
            'reason_cn': '本地商品别名命中。',
        }

    def modify_current_proposal(self) -> Optional[Dict[str, Any]]:
        related = self.products_from_related(self.sales_dialogue.get('related_products') or [])
        if related:
            primary = related[0]
            alternatives = related[1:]
            reply = self.build_sales_reply(
                need_text=self.need_text_cn(str(self.sales_dialogue.get('need') or 'unknown')),
                product=primary,
                alternatives=alternatives,
                reason=self.product_reason_for_need(primary, str(self.sales_dialogue.get('need') or 'unknown'), [], []),
            )
            return self.build_propose_decision(
                product=primary,
                alternatives=alternatives,
                need=str(self.sales_dialogue.get('need') or 'unknown'),
                reason='用户要求换一个，从备选商品中重新推荐。',
                reply=reply,
                confidence=0.9,
            )
        need = str(self.sales_dialogue.get('need') or '')
        if need:
            return self.decision_by_need_category(need, '')
        return None

    def modify_current_proposal_by_category(self, category: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        positive = list(category.get('positive_constraints') or [])
        negative = list(category.get('negative_constraints') or [])
        need = str(category.get('need_category') or self.sales_dialogue.get('need') or 'unknown')
        if need != 'unknown' or positive or negative:
            decision = self.decision_by_need_category(
                need,
                '',
                positive_constraints=positive,
                negative_constraints=negative,
                confidence=self.safe_float(category.get('confidence'), 0.8),
            )
            if decision is not None:
                return decision
        return self.modify_current_proposal()

    def products_from_related(self, related: List[Dict[str, Any]]) -> List[Product]:
        products: List[Product] = []
        for item in related:
            if not isinstance(item, dict):
                continue
            product = self.catalog.get(str(item.get('product_id') or ''))
            if product is not None:
                products.append(product)
        return products

    def need_text_cn(self, need_category: str) -> str:
        return {
            'thirsty': '有点口渴，想喝点东西',
            'drink': '想喝饮品',
            'hungry': '有点饿，想吃点东西',
            'snack': '想吃零食',
            'fruit': '想吃清爽健康的水果',
            'nutrition': '想补充营养',
            'energy': '想提神补充能量',
            'sleepy': '有点困，需要提神',
            'hygiene': '需要洗漱护理用品',
            'tissue': '需要纸巾或清洁用品',
            'clean': '需要清洁用品',
            'daily_goods': '需要日常生活用品',
            'explicit_product': 'explicit_product',
        }.get(need_category, '想选一件合适的商品')

    def product_reason_for_need(
        self,
        product: Product,
        need_category: str,
        positive_constraints: List[str],
        negative_constraints: List[str],
    ) -> str:
        del positive_constraints, negative_constraints
        if product.id == 'oreo':
            if need_category in ('hungry', 'snack'):
                return '它是甜味饼干，比普通小零食更顶饱，适合饿的时候快速补充能量'
            return '它口味经典，适合作为休闲零食'
        if product.id == 'chips':
            return '它咸香酥脆，比较解馋，适合想吃有味道零食的时候选择'
        if product.id == 'water_nongfu':
            return '它解渴直接、价格也低，适合口渴或者想要清爽饮品的时候选择'
        if product.id in ('cola_coca', 'cola_pepsi'):
            return '它是经典碳酸饮料，口感刺激，适合想喝可乐的时候选择'
        if product.id == 'milk_pure':
            return '它更健康，也能补充营养，适合不想喝碳酸饮料的时候选择'
        if product.id == 'tissue_vinda':
            return '它适合擦嘴、清洁和日常使用，是生活用品里很实用的选择'
        if product.id == 'redbull':
            return '它是功能饮料，适合困了、没精神或者想补充能量的时候选择'
        if product.id == 'coffee_nestle':
            return '它适合提神，比较适合学习或工作时想保持精神的时候选择'
        if product.id in ('orange', 'apple'):
            return '它清爽健康，价格也低，适合想吃水果或清淡零食的时候选择'
        if product.id in ('toothpaste', 'shampoo'):
            return '它属于常用生活护理用品，适合日常洗漱和清洁需求'
        if product.selling_points:
            return f'它符合您的需求，特点是{"、".join(product.selling_points[:2])}'
        return '它符合您当前的需求，也在当前可选商品范围内'

    def catalog_intro_reply(self) -> str:
        return '我这里有饮料、矿泉水、牛奶、零食、水果和生活用品。您可以说“我口渴了”“我有点饿”或者直接说商品名。'

    def reply_for_unclear_category(self, category: Dict[str, Any]) -> str:
        del category
        return (
            '我还没有完全判断出您想买哪类商品。'
            '我可以帮您推荐饮料、零食、水果、牛奶和生活用品。'
            '您可以换个说法，比如“我口渴了”“我有点饿”“我要纸巾”，'
            '或者直接问“你能做什么”。'
        )

    def normalize_cn(self, text: str) -> str:
        table = str.maketrans('', '', ' ，。！？!?、,. ')
        return text.strip().translate(table)

    def handle_sales_decision(
        self,
        task_id: str,
        text: str,
        decision: Dict[str, Any],
        command_meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        command_meta = command_meta or {}
        action = str(decision.get('action') or 'unknown').strip()
        confidence = self.safe_float(decision.get('confidence'), 0.0)
        product = self.product_from_decision(decision)
        reply = str(decision.get('reply_cn') or '').strip()
        if command_meta.get('source') == 'voice' and action == 'execute_pick' and not self.is_confirm_text(text):
            action = 'propose_product'
            decision['action'] = 'propose_product'
            decision['requires_confirmation'] = True
            if product is not None and not reply:
                decision['reply_cn'] = self.default_proposal_reply(product)
                reply = decision['reply_cn']

        if self.decision_has_unknown_product(decision):
            self.clear_sales_dialogue()
            self.say(task_id, '当前商品清单中没有该商品，请重新说明需要购买的商品。', priority=7)
            return

        if action == 'execute_pick':
            if product is None:
                self.say(task_id, '我还没有确定要购买的商品，请再说明一下。', priority=6)
                return
            self.clear_sales_dialogue()
            self.execute_b2_pick(task_id, product, text, 'llm_sales', confidence or 0.9, command_meta)
            return

        if action == 'propose_product':
            if product is None:
                self.update_sales_dialogue(
                    text,
                    decision,
                    state='asking_clarification',
                    waiting_for=str(decision.get('waiting_for') or WAIT_CLARIFY_PRODUCT),
                )
                self.say(task_id, reply or '我理解了您的需求，但还不能确定商品，请再说明一下。', priority=6)
                return
            self.update_sales_dialogue(
                text,
                decision,
                state='awaiting_confirmation',
                waiting_for=str(decision.get('waiting_for') or WAIT_CONFIRM_PRODUCT),
            )
            self.say(task_id, reply or self.default_proposal_reply(product), priority=7)
            return

        if action == 'ask_clarification':
            self.update_sales_dialogue(
                text,
                decision,
                state='asking_clarification',
                waiting_for=str(decision.get('waiting_for') or WAIT_CLARIFY_PRODUCT),
            )
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
        command_meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        command_meta = command_meta or {}
        task_request_id = str(command_meta.get('task_request_id') or task_id)
        if task_request_id in self.executed_task_request_ids:
            self.get_logger().info(f'Ignoring duplicate B-2 task_request_id={task_request_id}')
            return
        self.executed_task_request_ids.add(task_request_id)
        if len(self.executed_task_request_ids) > 100:
            self.executed_task_request_ids = set(list(self.executed_task_request_ids)[-80:])
        payload = {
            'schema_version': '1.0',
            'task_id': task_id,
            'task_request_id': task_request_id,
            'timestamp': time.time(),
            'source': source,
            'command_source': str(command_meta.get('source') or source),
            'session_id': str(command_meta.get('session_id') or ''),
            'confirm_utterance_id': str(command_meta.get('utterance_id') or ''),
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
            'pending_product_id': '',
            'pending_product_name': '',
            'waiting_for': '',
            'proposal_id': '',
            'need': '',
            'related_products': [],
            'pending_proposal': {},
            'constraints': {},
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

    def update_sales_dialogue(
        self,
        text: str,
        decision: Dict[str, Any],
        state: str,
        waiting_for: str = '',
    ) -> None:
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
        proposal_id = f"proposal_{int(time.time() * 1000)}" if product is not None and state == 'awaiting_confirmation' else ''

        self.sales_dialogue = {
            'active': state in ('awaiting_confirmation', 'asking_clarification'),
            'state': state,
            'history': history,
            'last_action': action,
            'last_product_id': product.id if product is not None else '',
            'last_product_name': product.name if product is not None else '',
            'pending_product_id': product.id if product is not None and state == 'awaiting_confirmation' else '',
            'pending_product_name': product.name if product is not None and state == 'awaiting_confirmation' else '',
            'waiting_for': waiting_for or (WAIT_CONFIRM_PRODUCT if product is not None and state == 'awaiting_confirmation' else ''),
            'proposal_id': proposal_id,
            'need': str(decision.get('need') or self.sales_dialogue.get('need') or ''),
            'related_products': related,
            'pending_proposal': {
                'main_product': product_to_dict(product) if product is not None else None,
                'alternatives': related,
                'reason': str(decision.get('reason_cn') or ''),
                'status': 'waiting_confirm',
                'source': 'voice' if decision.get('requires_confirmation') else '',
                'proposal_id': proposal_id,
                'constraints': dict(decision.get('constraints') or {}),
            } if product is not None and state == 'awaiting_confirmation' else {},
            'constraints': dict(decision.get('constraints') or {}),
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
            'pending_product_id': str(self.sales_dialogue.get('pending_product_id') or ''),
            'pending_product_name': str(self.sales_dialogue.get('pending_product_name') or ''),
            'waiting_for': str(self.sales_dialogue.get('waiting_for') or ''),
            'proposal_id': str(self.sales_dialogue.get('proposal_id') or ''),
            'related_products': self.sales_dialogue.get('related_products') or [],
            'pending_proposal': self.sales_dialogue.get('pending_proposal') or {},
            'last_reply': str(self.sales_dialogue.get('last_reply') or ''),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.sales_status_pub.publish(msg)

    def sales_dialogue_payload(self, current_source: str = '') -> Dict[str, Any]:
        return {
            'active': bool(self.sales_dialogue.get('active')),
            'state': str(self.sales_dialogue.get('state') or 'idle'),
            'waiting_for': str(self.sales_dialogue.get('waiting_for') or ''),
            'current_source': current_source,
            'conversation_history': list(self.sales_dialogue.get('history') or []),
            'last_action': str(self.sales_dialogue.get('last_action') or ''),
            'last_product_id': str(self.sales_dialogue.get('last_product_id') or ''),
            'last_product_name': str(self.sales_dialogue.get('last_product_name') or ''),
            'pending_product_id': str(self.sales_dialogue.get('pending_product_id') or ''),
            'pending_product_name': str(self.sales_dialogue.get('pending_product_name') or ''),
            'need': str(self.sales_dialogue.get('need') or ''),
            'related_products': list(self.sales_dialogue.get('related_products') or []),
            'pending_proposal': dict(self.sales_dialogue.get('pending_proposal') or {}),
            'constraints': dict(self.sales_dialogue.get('constraints') or {}),
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

    def is_confirm_text(self, text: str) -> bool:
        normalized = text.strip().replace(' ', '').replace('，', '').replace('。', '')
        return any(word in normalized for word in CONFIRM_WORDS)

    def is_weak_confirm_text(self, text: str) -> bool:
        normalized = self.normalize_cn(text)
        return normalized in WEAK_CONFIRM_WORDS

    def is_negative_text(self, text: str) -> bool:
        normalized = self.normalize_cn(text)
        return any(word in normalized for word in NEGATIVE_WORDS)

    def is_modify_text(self, text: str) -> bool:
        normalized = self.normalize_cn(text)
        return any(word in normalized for word in MODIFY_WORDS)

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
