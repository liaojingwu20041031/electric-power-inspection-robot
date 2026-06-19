import json
import os
import subprocess
import sys
import threading
import time
from functools import partial
from typing import Any, Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QHeaderView,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ylhb_interfaces.msg import CartState, SayText, TaskEvent, TaskStatus, VoiceStatus


TASK_IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png')
SYSTEM_MODES = ('sleep', 'ready', 'mapping', 'running', 'fault')


def workspace_path(*parts: str) -> str:
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_ws'))
    return os.path.join(workspace_dir, *parts)


def configure_input_method_environment() -> None:
    if os.getenv('ENABLE_CHINESE_IME', 'true') != 'true':
        return
    os.environ.setdefault('GTK_IM_MODULE', 'ibus')
    os.environ.setdefault('QT_IM_MODULE', 'ibus')
    os.environ.setdefault('XMODIFIERS', '@im=ibus')
    try:
        subprocess.Popen(
            ['ibus-daemon', '-drx'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        subprocess.run(
            ['ibus', 'engine', 'pinyin'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=1.0,
            check=False,
        )
    except FileNotFoundError:
        pass
    except subprocess.TimeoutExpired:
        pass


def system_mode_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class UiSignals(QObject):
    task_event = pyqtSignal(object)
    task_status = pyqtSignal(object)
    say_text = pyqtSignal(object)
    cart = pyqtSignal(object)
    voice_status = pyqtSignal(object)
    voice_session_status = pyqtSignal(object)
    text_command = pyqtSignal(object)
    sales_dialogue_status = pyqtSignal(object)
    localized_objects = pyqtSignal(object)
    system_status = pyqtSignal(object)
    system_mode = pyqtSignal(str)
    b1_result = pyqtSignal(bool, str)
    voice_capture_result = pyqtSignal(bool, str)
    ros_error = pyqtSignal(str)


class RetailDisplayRosBridge(Node):
    def __init__(self, signals: UiSignals) -> None:
        super().__init__('retail_display_ui_node')
        self.signals = signals

        self.declare_parameter('text_command_topic', '/retail_ai/text_command')
        self.declare_parameter('system_mode_topic', '/retail_ai/system_mode')
        self.declare_parameter('system_command_topic', '/retail_ai/system_command')
        self.declare_parameter('system_status_topic', '/retail_ai/system_status')
        self.declare_parameter('task_event_topic', '/retail_ai/task_event')
        self.declare_parameter('task_status_topic', '/retail_ai/task_status')
        self.declare_parameter('say_text_topic', '/retail_ai/say_text')
        self.declare_parameter('sales_dialogue_status_topic', '/retail_ai/sales_dialogue_status')
        self.declare_parameter('cart_topic', '/retail_ai/cart')
        self.declare_parameter('voice_status_topic', '/retail_ai/voice_status')
        self.declare_parameter('voice_session_status_topic', '/retail_ai/voice_session_status')
        self.declare_parameter('voice_command_event_topic', '/retail_ai/voice_command_event')
        self.declare_parameter('capture_voice_service_name', '/retail_ai/capture_voice')
        self.declare_parameter('start_voice_session_service_name', '/retail_ai/start_voice_session')
        self.declare_parameter('stop_voice_session_service_name', '/retail_ai/stop_voice_session')
        self.declare_parameter('localized_objects_topic', '/perception/localized_objects')
        self.declare_parameter('start_b1_service_name', '/retail_ai/start_b1_task')
        self.declare_parameter('task_image_dir', workspace_path('src', 'ylhb_llm', 'test_images'))
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_system_mode', 'ready')
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('display', ':0')
        self.declare_parameter('force_local_display', True)

        self.task_image_dir = os.path.expanduser(str(self.get_parameter('task_image_dir').value))
        self.fullscreen = bool(self.get_parameter('fullscreen').value)
        self.display = str(self.get_parameter('display').value)
        self.force_local_display = bool(self.get_parameter('force_local_display').value)

        self.text_pub = self.create_publisher(
            String, self.get_parameter('text_command_topic').value, 10)
        self.system_mode_pub = self.create_publisher(
            String, self.get_parameter('system_mode_topic').value, system_mode_qos())
        self.system_command_pub = self.create_publisher(
            String, self.get_parameter('system_command_topic').value, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.b1_client = self.create_client(
            Trigger, self.get_parameter('start_b1_service_name').value)
        self.capture_voice_client = self.create_client(
            Trigger, self.get_parameter('capture_voice_service_name').value)
        self.start_voice_session_client = self.create_client(
            Trigger, self.get_parameter('start_voice_session_service_name').value)
        self.stop_voice_session_client = self.create_client(
            Trigger, self.get_parameter('stop_voice_session_service_name').value)
        self.b1_service_ready = False

        self.create_subscription(TaskEvent, self.get_parameter('task_event_topic').value,
                                 lambda msg: self.signals.task_event.emit(msg), 10)
        self.create_subscription(TaskStatus, self.get_parameter('task_status_topic').value,
                                 lambda msg: self.signals.task_status.emit(msg), 10)
        self.create_subscription(SayText, self.get_parameter('say_text_topic').value,
                                 lambda msg: self.signals.say_text.emit(msg), 10)
        self.create_subscription(String, self.get_parameter('sales_dialogue_status_topic').value,
                                 lambda msg: self.signals.sales_dialogue_status.emit(msg), system_mode_qos())
        self.create_subscription(CartState, self.get_parameter('cart_topic').value,
                                 lambda msg: self.signals.cart.emit(msg), 10)
        self.create_subscription(VoiceStatus, self.get_parameter('voice_status_topic').value,
                                 lambda msg: self.signals.voice_status.emit(msg), 10)
        self.create_subscription(String, self.get_parameter('voice_session_status_topic').value,
                                 lambda msg: self.signals.voice_session_status.emit(msg), system_mode_qos())
        self.create_subscription(String, self.get_parameter('text_command_topic').value,
                                 lambda msg: self.signals.text_command.emit(msg), 10)
        self.create_subscription(String, self.get_parameter('localized_objects_topic').value,
                                 lambda msg: self.signals.localized_objects.emit(msg), 10)
        self.create_subscription(String, self.get_parameter('system_status_topic').value,
                                 lambda msg: self.signals.system_status.emit(msg), 10)
        self.create_subscription(String, self.get_parameter('system_mode_topic').value,
                                 lambda msg: self.signals.system_mode.emit(msg.data), system_mode_qos())
        self.create_timer(1.0, self.check_b1_service)

        initial_mode = str(self.get_parameter('initial_system_mode').value).strip()
        if initial_mode not in SYSTEM_MODES:
            initial_mode = 'ready'
        self.publish_system_mode(initial_mode)
        self.get_logger().info(
            f'Inspection display UI bridge started. initial_system_mode={initial_mode}'
        )

    def publish_text_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

    def publish_system_mode(self, mode: str) -> None:
        if mode not in SYSTEM_MODES:
            self.signals.ros_error.emit(f'未知系统模式: {mode}')
            return
        msg = String()
        msg.data = mode
        self.system_mode_pub.publish(msg)

    def publish_system_command(self, command: str, **kwargs: Any) -> None:
        payload = {'command': command}
        payload.update(kwargs)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.system_command_pub.publish(msg)

    def publish_zero_velocity(self, repeat: int = 5) -> None:
        twist = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

    def check_b1_service(self) -> None:
        self.b1_service_ready = self.b1_client.service_is_ready()

    def call_b1_service(self, wait_timeout_sec: float = 20.0) -> None:
        threading.Thread(
            target=self._call_b1_service_after_wait,
            args=(wait_timeout_sec,),
            daemon=True,
        ).start()

    def call_capture_voice_service(self, wait_timeout_sec: float = 2.0) -> None:
        threading.Thread(
            target=self._call_capture_voice_service_after_wait,
            args=(wait_timeout_sec,),
            daemon=True,
        ).start()

    def _call_b1_service_after_wait(self, wait_timeout_sec: float) -> None:
        deadline = time.monotonic() + wait_timeout_sec
        asked_supervisor = False
        while time.monotonic() < deadline:
            if self.b1_client.wait_for_service(timeout_sec=0.2):
                future = self.b1_client.call_async(Trigger.Request())
                future.add_done_callback(self._b1_done)
                return
            if not asked_supervisor:
                self.publish_system_command('start_llm')
                asked_supervisor = True
        self.signals.b1_result.emit(
            False,
            'B-1 服务未就绪：/retail_ai/start_b1_task。已尝试启动 AI 任务层，请检查 retail_task_node。',
        )

    def _b1_done(self, future: Any) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.signals.b1_result.emit(False, f'B-1 服务调用失败：{exc}')
            return
        self.signals.b1_result.emit(bool(result.success), str(result.message))

    def _call_capture_voice_service_after_wait(self, wait_timeout_sec: float) -> None:
        if not self.capture_voice_client.wait_for_service(timeout_sec=wait_timeout_sec):
            self.signals.voice_capture_result.emit(
                False,
                '语音输入服务未就绪：/retail_ai/capture_voice。请确认 AI 任务层已启动且 enable_voice:=true。',
            )
            return
        future = self.capture_voice_client.call_async(Trigger.Request())
        future.add_done_callback(self._capture_voice_done)

    def call_voice_session_service(self, start: bool, wait_timeout_sec: float = 2.0) -> None:
        client = self.start_voice_session_client if start else self.stop_voice_session_client
        threading.Thread(
            target=self._call_voice_session_service_after_wait,
            args=(client, start, wait_timeout_sec),
            daemon=True,
        ).start()

    def _call_voice_session_service_after_wait(self, client: Any, start: bool, wait_timeout_sec: float) -> None:
        if not client.wait_for_service(timeout_sec=wait_timeout_sec):
            action = '开启' if start else '关闭'
            self.signals.ros_error.emit(f'语音模式{action}服务未就绪。')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda fut: self._voice_session_done(fut, start))

    def _voice_session_done(self, future: Any, start: bool) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.signals.ros_error.emit(f'语音模式服务调用失败：{exc}')
            return
        if not result.success:
            self.signals.ros_error.emit(str(result.message))
            return
        self.get_logger().info(f'Voice session {"started" if start else "stopped"}: {result.message}')

    def _capture_voice_done(self, future: Any) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.signals.voice_capture_result.emit(False, f'语音输入服务调用失败：{exc}')
            return
        self.signals.voice_capture_result.emit(bool(result.success), str(result.message))


class RetailDisplayWindow(QWidget):
    def __init__(self, bridge: RetailDisplayRosBridge, signals: UiSignals) -> None:
        super().__init__()
        self.bridge = bridge
        self.signals = signals
        self.system_mode = 'ready'
        self.task_phase = 'idle'
        self.current_task_id = ''
        self.last_update_ts = 0.0
        self.voice_capture_active = False
        self.voice_session_enabled = False
        self.voice_session_state = 'OFF'
        self.voice_speaking = False
        self.latest_task_image = ''
        self.latest_objects_payload: Dict[str, Any] = {}
        self.sales_dialogue_payload: Dict[str, Any] = {}
        self.objects_dirty = False
        self.cached_pixmap_path = ''
        self.cached_pixmap = QPixmap()
        self.compact_ui = self.detect_compact_ui()

        self.setWindowTitle('电力巡检机器人总控台')
        self.resize_to_screen()
        self.build_ui()
        self.apply_style()
        self.connect_signals()
        self.set_mode(str(self.bridge.get_parameter('initial_system_mode').value), publish=False)

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self.refresh_time_label)
        self.clock_timer.start(1000)
        self.service_timer = QTimer(self)
        self.service_timer.timeout.connect(self.refresh_service_label)
        self.service_timer.start(1000)
        self.object_timer = QTimer(self)
        self.object_timer.timeout.connect(self.refresh_objects_table)
        self.object_timer.start(200)

    def detect_compact_ui(self) -> bool:
        screen = QApplication.primaryScreen()
        if screen is None:
            return False
        size = screen.availableGeometry().size()
        return size.width() < 1180 or size.height() < 760

    def resize_to_screen(self) -> None:
        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1280, 820)
            return
        available = screen.availableGeometry()
        width = min(1280, max(360, available.width()))
        height = min(820, max(320, available.height()))
        self.resize(width, height)

    def configure_layout(self, layout: Any) -> None:
        margin = 6 if self.compact_ui else 10
        spacing = 6 if self.compact_ui else 10
        layout.setContentsMargins(margin, margin, margin, margin)
        layout.setSpacing(spacing)

    def configure_table(self, table: QTableWidget) -> None:
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setWordWrap(False)

    def build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.NoFrame)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        self.content_widget = QWidget()
        root = QVBoxLayout(self.content_widget)
        self.configure_layout(root)
        root.addLayout(self.build_status_bar())

        body = QVBoxLayout() if self.compact_ui else QHBoxLayout()
        self.configure_layout(body)
        body.addWidget(self.build_task_panel(), 2)
        body.addWidget(self.build_timeline_panel(), 3)
        body.addWidget(self.build_recognition_panel(), 3)
        root.addLayout(body, 1)
        root.addWidget(self.build_bottom_panel(), 2)

        self.scroll_area.setWidget(self.content_widget)
        outer.addWidget(self.scroll_area)

    def build_status_bar(self) -> Any:
        layout = QGridLayout() if self.compact_ui else QHBoxLayout()
        self.configure_layout(layout)
        self.title_label = QLabel('电力巡检驾驶舱 / Inspection Control')
        self.ros_label = QLabel('ROS: connected')
        self.mode_label = QLabel()
        self.task_label = QLabel('当前任务: -')
        self.b1_service_label = QLabel('B1服务: 检查中')
        self.voice_label = QLabel('语音: 空闲')
        self.time_label = QLabel()
        self.software_stop_button = QPushButton('软件急停')
        self.software_stop_button.setObjectName('dangerButton')
        self.software_stop_button.clicked.connect(self.software_stop)
        self.complete_button = QPushButton('任务完成，返回准备')
        self.complete_button.clicked.connect(self.return_ready_after_done)
        self.fullscreen_button = QPushButton('退出全屏')
        self.fullscreen_button.clicked.connect(self.toggle_fullscreen)

        self.title_label.setObjectName('titleLabel')
        self.mode_label.setObjectName('modeBadge')
        for widget in (self.ros_label, self.mode_label, self.task_label, self.b1_service_label,
                       self.voice_label, self.time_label):
            widget.setFrameShape(QFrame.Panel)
            widget.setFrameShadow(QFrame.Sunken)
            widget.setMinimumHeight(28 if self.compact_ui else 34)
            widget.setWordWrap(self.compact_ui)

        if self.compact_ui:
            layout.addWidget(self.title_label, 0, 0, 1, 4)
            layout.addWidget(self.ros_label, 1, 0)
            layout.addWidget(self.mode_label, 1, 1)
            layout.addWidget(self.b1_service_label, 1, 2)
            layout.addWidget(self.voice_label, 1, 3)
            layout.addWidget(self.task_label, 2, 0, 1, 2)
            layout.addWidget(self.time_label, 2, 2)
            layout.addWidget(self.fullscreen_button, 2, 3)
            layout.addWidget(self.complete_button, 3, 0, 1, 2)
            layout.addWidget(self.software_stop_button, 3, 2, 1, 2)
        else:
            layout.addWidget(self.title_label, 2)
            layout.addWidget(self.ros_label)
            layout.addWidget(self.mode_label)
            layout.addWidget(self.task_label, 2)
            layout.addWidget(self.b1_service_label)
            layout.addWidget(self.voice_label)
            layout.addWidget(self.time_label)
            layout.addWidget(self.fullscreen_button)
            layout.addWidget(self.complete_button)
            layout.addWidget(self.software_stop_button)
        return layout

    def build_task_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.configure_layout(layout)

        mode_box = QGroupBox('系统状态')
        mode_layout = QGridLayout(mode_box)
        for idx, (mode, text) in enumerate((
            ('sleep', '休眠'),
            ('ready', '运行准备'),
            ('mapping', '建图模式'),
            ('fault', '异常待处理'),
        )):
            button = QPushButton(text)
            button.clicked.connect(partial(self.set_mode, mode, True))
            mode_layout.addWidget(button, idx // 2, idx % 2)
        layout.addWidget(mode_box)

        task_a = QGroupBox('任务 A-1 基本运动')
        task_a_layout = QGridLayout(task_a)
        for idx, command in enumerate(('前进', '后退', '左转', '右转', '停止')):
            button = QPushButton(command)
            button.clicked.connect(partial(self.send_motion_command, command))
            task_a_layout.addWidget(button, idx // 2, idx % 2)
        layout.addWidget(task_a)

        task_a2 = QGroupBox('任务 A-2 现场语音交互')
        task_a2_layout = QVBoxLayout(task_a2)
        self.configure_layout(task_a2_layout)
        self.voice_session_button = QPushButton('开启语音模式')
        self.voice_session_button.clicked.connect(self.toggle_voice_session)
        task_a2_layout.addWidget(self.voice_session_button)
        self.voice_session_state_label = QLabel('状态: 关闭')
        self.voice_session_wake_label = QLabel('唤醒词: 小零小零')
        self.voice_session_asr_label = QLabel('ASR: -')
        self.voice_session_text_label = QLabel('入口: -')
        for label in (
            self.voice_session_state_label,
            self.voice_session_wake_label,
            self.voice_session_asr_label,
            self.voice_session_text_label,
        ):
            label.setFrameShape(QFrame.Panel)
            label.setFrameShadow(QFrame.Sunken)
            label.setMinimumHeight(24 if self.compact_ui else 28)
            label.setWordWrap(True)
        task_a2_layout.addWidget(self.voice_session_state_label)
        task_a2_layout.addWidget(self.voice_session_wake_label)
        task_a2_layout.addWidget(self.voice_session_asr_label)
        task_a2_layout.addWidget(self.voice_session_text_label)
        layout.addWidget(task_a2)

        task_b = QGroupBox('任务 B')
        task_b_layout = QVBoxLayout(task_b)
        self.b1_button = QPushButton('导入任务书图片 / 开始 B-1')
        self.b1_button.clicked.connect(self.start_b1)
        task_b_layout.addWidget(self.b1_button)

        b2_box = QGroupBox('B-2 销售对话')
        b2_layout = QVBoxLayout(b2_box)
        self.configure_layout(b2_layout)
        b2_row = QHBoxLayout()
        self.shopping_input = QLineEdit()
        self.shopping_input.setPlaceholderText('例如：来瓶可乐 / 我口渴了 / 想买个便宜点的喝的 / 不要碳酸的')
        self.shopping_input.returnPressed.connect(self.start_b2)
        self.b2_button = QPushButton('发送')
        self.b2_button.clicked.connect(self.start_b2)
        self.voice_input_button = QPushButton('语音模式')
        self.voice_input_button.clicked.connect(self.toggle_voice_session)
        b2_row.addWidget(self.shopping_input, 1)
        b2_row.addWidget(self.b2_button)
        b2_row.addWidget(self.voice_input_button)
        b2_layout.addLayout(b2_row)
        self.sales_state_label = QLabel('状态: 空闲')
        self.sales_primary_label = QLabel('主推: -')
        self.sales_related_label = QLabel('备选: -')
        self.sales_reply_label = QLabel('机器人: -')
        self.sales_reply_label.setWordWrap(True)
        for label in (
            self.sales_state_label,
            self.sales_primary_label,
            self.sales_related_label,
            self.sales_reply_label,
        ):
            label.setFrameShape(QFrame.Panel)
            label.setFrameShadow(QFrame.Sunken)
            label.setMinimumHeight(24 if self.compact_ui else 28)
        b2_layout.addWidget(self.sales_state_label)
        b2_layout.addWidget(self.sales_primary_label)
        b2_layout.addWidget(self.sales_related_label)
        b2_layout.addWidget(self.sales_reply_label)
        task_b_layout.addWidget(b2_box)
        layout.addWidget(task_b)

        task_c = QGroupBox('任务 C 巡检记录')
        task_c_layout = QVBoxLayout(task_c)
        self.checkout_button = QPushButton('生成巡检记录')
        self.checkout_button.clicked.connect(self.start_checkout)
        task_c_layout.addWidget(self.checkout_button)
        layout.addWidget(task_c)

        task_d = QGroupBox('任务 D 创意展示')
        task_d_layout = QVBoxLayout(task_d)
        self.demo_button = QPushButton('打开电力巡检驾驶舱')
        self.demo_button.clicked.connect(lambda: self.tabs.setCurrentIndex(1))
        task_d_layout.addWidget(self.demo_button)
        layout.addWidget(task_d)
        layout.addStretch(1)
        return panel

    def build_timeline_panel(self) -> QWidget:
        self.tabs = QTabWidget()

        timeline_page = QWidget()
        timeline_layout = QVBoxLayout(timeline_page)
        self.configure_layout(timeline_layout)
        self.timeline = QListWidget()
        timeline_layout.addWidget(self.timeline)
        self.tabs.addTab(timeline_page, '任务流')

        system_page = QWidget()
        system_layout = QVBoxLayout(system_page)
        self.configure_layout(system_layout)
        system_layout.addWidget(self.build_system_control_page())
        self.tabs.addTab(system_page, '系统控制')

        cockpit_page = QWidget()
        cockpit_layout = QVBoxLayout(cockpit_page)
        self.configure_layout(cockpit_layout)
        self.cockpit_text = QPlainTextEdit()
        self.cockpit_text.setReadOnly(True)
        self.cockpit_text.document().setMaximumBlockCount(100)
        self.cockpit_text.setPlainText(
            '电力巡检驾驶舱\n'
            '路线巡检、检查点识别、告警、语音播报和任务状态会在真实 ROS 链路中更新。\n'
            '本页面不伪造检测结果、不绕过任务事件。'
        )
        guide_row = QHBoxLayout()
        guide_b1 = QPushButton('引导 B-1')
        guide_b2 = QPushButton('引导 B-2')
        guide_c = QPushButton('引导 C')
        guide_b1.clicked.connect(self.start_b1)
        guide_b2.clicked.connect(self.start_b2)
        guide_c.clicked.connect(self.start_checkout)
        for button in (guide_b1, guide_b2, guide_c):
            guide_row.addWidget(button)
        cockpit_layout.addLayout(guide_row)
        cockpit_layout.addWidget(self.cockpit_text)
        self.tabs.addTab(cockpit_page, '电力巡检驾驶舱')
        return self.tabs

    def build_system_control_page(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.configure_layout(layout)

        self.system_status_table = QTableWidget(6, 2)
        self.system_status_table.setHorizontalHeaderLabels(['模块', '状态'])
        self.configure_table(self.system_status_table)
        for row, name in enumerate(('bringup', 'mapping', 'navigation', 'zed', 'perception', 'llm')):
            self.system_status_table.setItem(row, 0, QTableWidgetItem(name))
            self.system_status_table.setItem(row, 1, QTableWidgetItem('unknown'))
        layout.addWidget(self.system_status_table)

        competition_box = QGroupBox('巡检一键控制')
        competition_layout = QGridLayout(competition_box)
        self.add_system_button(competition_layout, 0, 0, '一键启动巡检节点', 'start_competition_stack')
        self.add_system_button(competition_layout, 0, 1, '一键停止巡检节点', 'stop_competition_stack', mode='ready')
        layout.addWidget(competition_box)

        mapping_box = QGroupBox('建图控制')
        mapping_layout = QGridLayout(mapping_box)
        self.add_system_button(mapping_layout, 0, 0, '启动建图', 'start_mapping', mode='mapping')
        self.add_system_button(mapping_layout, 0, 1, '停止建图', 'stop_mapping', mode='ready')
        save_button = QPushButton('保存地图')
        save_button.clicked.connect(self.save_map)
        mapping_layout.addWidget(save_button, 1, 0)
        self.add_system_button(mapping_layout, 1, 1, '启动底盘/雷达', 'start_bringup')
        self.add_system_button(mapping_layout, 2, 0, '停止底盘/雷达', 'stop_bringup')
        layout.addWidget(mapping_box)

        nav_box = QGroupBox('导航 / 感知 / AI')
        nav_layout = QGridLayout(nav_box)
        self.add_system_button(nav_layout, 0, 0, '启动导航', 'start_navigation')
        self.add_system_button(nav_layout, 0, 1, '停止导航', 'stop_navigation')
        self.add_system_button(nav_layout, 1, 0, '启动 ZED', 'start_zed')
        self.add_system_button(nav_layout, 1, 1, '停止 ZED', 'stop_zed')
        self.add_system_button(nav_layout, 2, 0, '启动感知', 'start_perception')
        self.add_system_button(nav_layout, 2, 1, '重启感知', 'restart_perception')
        self.add_system_button(nav_layout, 3, 0, '启动 AI 任务层', 'start_llm')
        self.add_system_button(nav_layout, 3, 1, '停止 AI 任务层', 'stop_llm')
        layout.addWidget(nav_box)

        self.system_log = QPlainTextEdit()
        self.system_log.setReadOnly(True)
        self.system_log.document().setMaximumBlockCount(100)
        layout.addWidget(self.system_log, 1)
        return panel

    def add_system_button(
        self,
        layout: QGridLayout,
        row: int,
        col: int,
        text: str,
        command: str,
        mode: str = '',
    ) -> None:
        button = QPushButton(text)
        button.clicked.connect(partial(self.send_system_command, command, mode))
        layout.addWidget(button, row, col)

    def build_recognition_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.configure_layout(layout)
        self.image_label = QLabel('任务书图片预览')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(120 if self.compact_ui else 180)
        self.image_label.setFrameShape(QFrame.Box)
        layout.addWidget(self.image_label)

        self.objects_table = QTableWidget(0, 4)
        self.objects_table.setHorizontalHeaderLabels(['目标/类别', '置信度', '数量', '位置摘要'])
        self.configure_table(self.objects_table)
        layout.addWidget(self.objects_table, 1)
        return panel

    def build_bottom_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel) if self.compact_ui else QHBoxLayout(panel)
        self.configure_layout(layout)
        speech_box = QGroupBox('播报文本')
        speech_layout = QVBoxLayout(speech_box)
        self.configure_layout(speech_layout)
        self.say_text_view = QPlainTextEdit()
        self.say_text_view.setReadOnly(True)
        self.say_text_view.document().setMaximumBlockCount(100)
        speech_layout.addWidget(self.say_text_view)
        layout.addWidget(speech_box, 2)

        cart_box = QGroupBox('巡检结果 / 告警')
        cart_layout = QVBoxLayout(cart_box)
        self.configure_layout(cart_layout)
        self.cart_table = QTableWidget(0, 4)
        self.cart_table.setHorizontalHeaderLabels(['目标', '数量', '等级', '摘要'])
        self.configure_table(self.cart_table)
        self.total_label = QLabel('告警数: 0')
        cart_layout.addWidget(self.cart_table)
        cart_layout.addWidget(self.total_label)
        layout.addWidget(cart_box, 3)
        return panel

    def connect_signals(self) -> None:
        self.signals.task_event.connect(self.on_task_event)
        self.signals.task_status.connect(self.on_task_status)
        self.signals.say_text.connect(self.on_say_text)
        self.signals.sales_dialogue_status.connect(self.on_sales_dialogue_status)
        self.signals.cart.connect(self.on_cart)
        self.signals.voice_status.connect(self.on_voice_status)
        self.signals.voice_session_status.connect(self.on_voice_session_status)
        self.signals.text_command.connect(self.on_text_command)
        self.signals.localized_objects.connect(self.on_localized_objects)
        self.signals.system_status.connect(self.on_system_status)
        self.signals.system_mode.connect(self.on_system_mode)
        self.signals.b1_result.connect(self.on_b1_result)
        self.signals.voice_capture_result.connect(self.on_voice_capture_result)
        self.signals.ros_error.connect(self.show_error)

    def apply_style(self) -> None:
        font_size = 12 if self.compact_ui else 14
        title_size = 15 if self.compact_ui else 18
        button_padding = '5px 8px' if self.compact_ui else '8px 12px'
        button_min_height = 22 if self.compact_ui else 26
        header_padding = 4 if self.compact_ui else 6
        tab_padding = '6px 10px' if self.compact_ui else '8px 14px'
        self.setStyleSheet("""
            QWidget {
                background: #f5f7fb;
                color: #172033;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
                font-size: %(font_size)dpx;
            }
            QGroupBox {
                border: 1px solid #cfd7e6;
                border-radius: 8px;
                margin-top: 10px;
                padding: 10px;
                background: #ffffff;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background: #2557d6;
                color: white;
                border: 0;
                border-radius: 6px;
                padding: %(button_padding)s;
                min-height: %(button_min_height)dpx;
                font-weight: 600;
            }
            QPushButton:hover { background: #1f49b8; }
            QPushButton:disabled {
                background: #aeb8cc;
                color: #eef2f8;
            }
            QPushButton#dangerButton {
                background: #d92d20;
                font-size: 16px;
            }
            QPushButton#dangerButton:hover { background: #b42318; }
            QLabel#titleLabel {
                background: #111827;
                color: #f9fafb;
                border-radius: 6px;
                padding: %(button_padding)s;
                font-size: %(title_size)dpx;
                font-weight: 700;
            }
            QLabel#modeBadge {
                border-radius: 6px;
                padding: 6px 10px;
                font-weight: 700;
            }
            QTableWidget, QListWidget, QPlainTextEdit, QLineEdit {
                background: #ffffff;
                border: 1px solid #d7deea;
                border-radius: 6px;
                selection-background-color: #dbe7ff;
                color: #172033;
            }
            QHeaderView::section {
                background: #e9eef8;
                padding: %(header_padding)dpx;
                border: 0;
                font-weight: 700;
            }
            QTabWidget::pane {
                border: 1px solid #cfd7e6;
                border-radius: 8px;
                background: #ffffff;
            }
            QTabBar::tab {
                background: #e9eef8;
                padding: %(tab_padding)s;
                margin-right: 2px;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
            }
            QTabBar::tab:selected {
                background: #2557d6;
                color: white;
            }
        """ % {
            'font_size': font_size,
            'title_size': title_size,
            'button_padding': button_padding,
            'button_min_height': button_min_height,
            'header_padding': header_padding,
            'tab_padding': tab_padding,
        })

    def set_mode(self, mode: str, publish: bool = True) -> None:
        if mode not in SYSTEM_MODES:
            return
        self.system_mode = mode
        if mode in ('sleep', 'ready', 'mapping'):
            self.task_phase = 'idle'
        if publish:
            self.bridge.publish_system_mode(mode)
        self.refresh_controls()
        self.add_timeline(f'系统状态 -> {self.mode_text(mode)}')

    def on_system_mode(self, mode: str) -> None:
        mode = mode.strip()
        if mode in SYSTEM_MODES and mode != self.system_mode:
            self.set_mode(mode, publish=False)

    def refresh_controls(self) -> None:
        can_start = self.system_mode == 'ready'
        for button in (self.b1_button, self.b2_button, self.checkout_button):
            button.setEnabled(can_start)
        voice_button_enabled = can_start or self.voice_session_enabled
        self.voice_input_button.setEnabled(voice_button_enabled and not self.voice_capture_active)
        self.voice_session_button.setEnabled(voice_button_enabled and not self.voice_capture_active)
        session_text = '关闭语音模式' if self.voice_session_enabled else '开启语音模式'
        if self.voice_capture_active:
            session_text = '录音中'
        self.voice_session_button.setText(session_text)
        self.voice_input_button.setText(session_text if self.voice_session_enabled else '语音模式')
        self.complete_button.setEnabled(
            self.system_mode == 'running' and self.task_phase == 'completed'
        )
        self.mode_label.setText(f'系统: {self.mode_text(self.system_mode)}')
        colors = {
            'sleep': '#667085',
            'ready': '#079455',
            'mapping': '#1570ef',
            'running': '#dc6803',
            'fault': '#d92d20',
        }
        self.mode_label.setStyleSheet(
            f'background: {colors.get(self.system_mode, "#667085")}; color: white;'
        )

    def mode_text(self, mode: str) -> str:
        return {
            'sleep': '休眠',
            'ready': '运行准备',
            'mapping': '手动建图',
            'running': '任务运行',
            'fault': '异常待处理',
        }.get(mode, mode)

    def confirm_start(self, title: str, detail: str) -> bool:
        if self.system_mode != 'ready':
            self.show_error('当前系统不在运行准备状态，不能启动新任务。')
            return False
        return QMessageBox.question(
            self,
            title,
            detail,
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) == QMessageBox.Yes

    def send_motion_command(self, command: str) -> None:
        if command != '停止' and self.system_mode not in ('ready', 'mapping', 'running'):
            self.show_error('当前状态不允许运动控制。')
            return
        self.bridge.publish_text_command(command)
        self.add_timeline(f'A-1 运动指令: {command}')

    def start_b1(self) -> None:
        ok, message, image_path = self.find_unique_task_image()
        if not ok:
            self.show_error(message)
            return
        self.latest_task_image = image_path
        self.show_task_image(image_path)
        if not self.confirm_start('确认启动 B-1', f'使用图片：\n{image_path}\n\n启动图片推荐任务？'):
            return
        self.task_phase = 'executing'
        self.set_mode('running', publish=True)
        self.bridge.call_b1_service()

    def start_b2(self) -> None:
        text = self.shopping_input.text().strip()
        if not text:
            self.show_error('请输入巡检指令、检查点或检测目标。')
            return
        if self.system_mode != 'ready':
            self.show_error('当前系统不在运行准备状态，不能发送 B-2 销售对话。')
            return
        self.bridge.publish_text_command(text)
        self.add_timeline(f'B-2 销售对话: {text}')

    def capture_voice(self) -> None:
        if self.system_mode != 'ready':
            self.show_error('当前系统不在运行准备状态，不能录入语音指令。')
            return
        if self.voice_speaking:
            self.show_error('机器人正在播报，请等播报结束后再录音。')
            return
        self.voice_capture_active = True
        self.refresh_controls()
        self.add_timeline('语音输入: 开始录音识别')
        self.bridge.call_capture_voice_service()

    def toggle_voice_session(self) -> None:
        if self.system_mode != 'ready' and not self.voice_session_enabled:
            self.show_error('当前系统不在运行准备状态，不能开启语音模式。')
            return
        self.bridge.call_voice_session_service(not self.voice_session_enabled)
        self.add_timeline('语音模式: 请求关闭' if self.voice_session_enabled else '语音模式: 请求开启')

    def start_checkout(self) -> None:
        if not self.confirm_start('确认启动 C', '生成巡检记录并汇总当前检测结果？'):
            return
        self.task_phase = 'executing'
        self.set_mode('running', publish=True)
        self.bridge.publish_text_command('一共多少钱')
        self.add_timeline('C 巡检记录指令: 汇总当前检测结果')

    def software_stop(self) -> None:
        self.set_mode('fault', publish=True)
        self.bridge.publish_text_command('停止')
        self.bridge.publish_system_command('emergency_stop')
        threading.Thread(target=self.bridge.publish_zero_velocity, daemon=True).start()
        self.add_timeline('软件急停: system_mode=fault, text_command=停止, /cmd_vel=0')

    def return_ready_after_done(self) -> None:
        if self.system_mode == 'running' and self.task_phase == 'completed':
            self.set_mode('ready', publish=True)

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.setWindowFlag(Qt.FramelessWindowHint, False)
            self.showNormal()
            self.fullscreen_button.setText('进入全屏')
        else:
            self.setWindowFlag(Qt.FramelessWindowHint, True)
            self.showFullScreen()
            screen = QApplication.primaryScreen()
            if screen is not None:
                self.setGeometry(screen.availableGeometry())
            self.fullscreen_button.setText('退出全屏')

    def send_system_command(self, command: str, mode: str = '') -> None:
        self.bridge.publish_system_command(command)
        if mode:
            self.set_mode(mode, publish=True)
        self.add_timeline(f'系统命令: {command}')

    def save_map(self) -> None:
        default_name = time.strftime('inspection_map_%Y%m%d_%H%M')
        map_name, ok = QInputDialog.getText(self, '保存地图', '请输入地图名称：', text=default_name)
        if not ok:
            return
        self.bridge.publish_system_command('save_map', map_name=map_name.strip() or default_name)
        self.add_timeline(f'系统命令: save_map {map_name.strip() or default_name}')

    def find_unique_task_image(self) -> Tuple[bool, str, str]:
        image_dir = self.bridge.task_image_dir
        if not os.path.isdir(image_dir):
            return False, f'任务书图片目录不存在：{image_dir}', ''
        candidates = [
            os.path.join(image_dir, name)
            for name in sorted(os.listdir(image_dir))
            if os.path.isfile(os.path.join(image_dir, name))
            and name.lower().endswith(TASK_IMAGE_EXTENSIONS)
        ]
        if not candidates:
            return False, '未找到任务书图片，请只保留一张 jpg/jpeg/png 图片。', ''
        if len(candidates) > 1:
            return False, '目录内存在多张图片，请只保留一张 jpg/jpeg/png 图片。', ''
        return True, '', candidates[0]

    def show_task_image(self, path: str) -> None:
        if self.cached_pixmap_path != path:
            self.cached_pixmap = QPixmap(path)
            self.cached_pixmap_path = path
        if self.cached_pixmap.isNull():
            self.image_label.setText(f'无法预览图片\n{path}')
            return
        self.image_label.setPixmap(
            self.cached_pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def on_b1_result(self, success: bool, message: str) -> None:
        payload = self.parse_json_message(message)
        if success:
            self.current_task_id = str(payload.get('task_id') or self.current_task_id)
            self.task_label.setText(f'当前任务: {self.current_task_id}')
            self.add_timeline(f"B-1 已启动: {payload.get('say_text', '')}")
        else:
            self.task_phase = 'idle'
            self.set_mode('ready', publish=True)
            self.show_error(str(payload.get('error') or message))

    def on_voice_capture_result(self, success: bool, message: str) -> None:
        self.voice_capture_active = False
        self.refresh_controls()
        if success:
            text = message.strip()
            self.shopping_input.setText(text)
            self.add_timeline(f'语音识别: {text}')
        else:
            self.show_error(message)

    def on_voice_session_status(self, msg: String) -> None:
        payload = self.parse_json_message(msg.data)
        self.voice_session_enabled = bool(payload.get('enabled'))
        self.voice_session_state = str(payload.get('state') or 'OFF')
        state_text = {
            'OFF': '关闭',
            'WAIT_WAKE': '待唤醒',
            'LISTENING': '监听中',
            'RECORDING': '录音中',
            'ASR_PROCESSING': '识别中',
            'AWAKENED_IDLE': '已唤醒',
            'TTS_PAUSED': '播报暂停监听',
        }.get(self.voice_session_state, self.voice_session_state)
        if payload.get('awakened'):
            state_text = f'{state_text} / 已唤醒'
        self.voice_session_state_label.setText(f'状态: {state_text}')
        self.voice_session_wake_label.setText(f"唤醒词: {payload.get('wake_phrase') or '小零小零'}")
        self.voice_session_asr_label.setText(f"ASR: {payload.get('last_asr_text') or '-'}")
        self.voice_session_text_label.setText(f"入口: {payload.get('last_published_text') or '-'}")
        self.refresh_controls()
        self.touch_update()

    def on_text_command(self, msg: String) -> None:
        text, _payload = self.text_from_command_message(msg.data)
        self.voice_session_text_label.setText(f'入口: {text or "-"}')

    def text_from_command_message(self, data: str) -> Tuple[str, Dict[str, Any]]:
        raw = data.strip()
        if not raw.startswith('{'):
            return raw, {}
        payload = self.parse_json_message(raw)
        return str(payload.get('text') or raw), payload

    def on_task_event(self, msg: TaskEvent) -> None:
        self.current_task_id = msg.task_id
        self.task_label.setText(f'当前任务: {msg.task_id}')
        raw = self.parse_json_message(msg.raw_json)
        detail = raw.get('reason') or raw.get('next_step') or ''
        self.add_timeline(
            f'TaskEvent {msg.intent}: {msg.item_name or "-"} -> {msg.destination or "-"} {detail}'
        )
        if self.system_mode == 'ready':
            self.task_phase = 'executing'
            self.set_mode('running', publish=True)

    def on_task_status(self, msg: TaskStatus) -> None:
        self.add_timeline(
            f'TaskStatus {msg.status}: task={msg.task_id or "-"} stage={msg.stage or "-"} '
            f'reason={msg.reason or "-"}'
        )
        if msg.status in ('failed', 'rejected'):
            self.task_phase = 'failed'
            self.set_mode('fault', publish=True)
            return
        if msg.status in ('completed',):
            self.task_phase = 'completed'
        elif msg.status == 'succeeded' and msg.stage in (
            'place', 'return_start', 'return_to_start', 'start', 'arrive_start'
        ):
            self.task_phase = 'completed'
        self.refresh_controls()

    def on_say_text(self, msg: SayText) -> None:
        self.say_text_view.appendPlainText(f'[{msg.task_id}] {msg.text}')
        self.add_cockpit_line(f'播报: {msg.text}')
        self.touch_update()

    def on_sales_dialogue_status(self, msg: String) -> None:
        payload = self.parse_json_message(msg.data)
        self.sales_dialogue_payload = payload
        state = str(payload.get('state') or 'idle')
        state_text = {
            'idle': '空闲',
            'awaiting_confirmation': '等待确认',
            'asking_clarification': '需要补充',
        }.get(state, state)
        primary_name = str(payload.get('primary_product_name') or '')
        primary_price = float(payload.get('primary_price', 0.0) or 0.0)
        if primary_name:
            primary = f'{primary_name} {self.format_price(primary_price)}元'
        else:
            primary = '-'
        related_items = []
        for item in payload.get('related_products') or []:
            if not isinstance(item, dict):
                continue
            name = str(item.get('product_name') or '')
            if not name:
                continue
            price = self.format_price(float(item.get('price', 0.0) or 0.0))
            related_items.append(f'{name} {price}元')
        related = ' / '.join(related_items) if related_items else '-'
        reply = str(payload.get('last_reply') or '-')

        self.sales_state_label.setText(f'状态: {state_text}')
        self.sales_primary_label.setText(f'主推: {primary}')
        self.sales_related_label.setText(f'备选: {related}')
        self.sales_reply_label.setText(f'机器人: {reply}')
        self.add_cockpit_line(f'销售对话: {state_text} 主推={primary} 备选={related}')
        self.touch_update()

    def on_cart(self, msg: CartState) -> None:
        self.cart_table.setRowCount(len(msg.items))
        for row, item in enumerate(msg.items):
            subtotal = int(item.quantity) * float(item.unit_price)
            values = [item.name, str(item.quantity), self.format_price(item.unit_price),
                      self.format_price(subtotal)]
            for col, value in enumerate(values):
                self.cart_table.setItem(row, col, QTableWidgetItem(value))
        self.total_label.setText(f'告警数: {len(msg.items)}')
        self.add_cockpit_line(f'巡检结果更新: {len(msg.items)} 类目标/告警')
        self.touch_update()

    def on_voice_status(self, msg: VoiceStatus) -> None:
        self.voice_speaking = bool(msg.speaking)
        self.voice_label.setText('语音: 播报中' if msg.speaking else '语音: 空闲')
        self.refresh_controls()
        self.touch_update()

    def on_localized_objects(self, msg: String) -> None:
        try:
            self.latest_objects_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.show_error('收到无法解析的 /perception/localized_objects JSON。')
            return
        self.objects_dirty = True
        self.touch_update()

    def refresh_objects_table(self) -> None:
        if not self.objects_dirty:
            return
        self.objects_dirty = False
        objects = self.latest_objects_payload.get('objects', [])
        if not isinstance(objects, list):
            objects = []
        grouped: Dict[str, Dict[str, Any]] = {}
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            name = str(obj.get('class_name') or obj.get('label') or obj.get('name') or obj.get('item_name') or '-')
            entry = grouped.setdefault(name, {'count': 0, 'confidence': 0.0, 'raw': obj})
            entry['count'] += 1
            entry['confidence'] = max(float(obj.get('confidence', 0.0) or 0.0), entry['confidence'])
        self.objects_table.setRowCount(len(grouped))
        for row, (name, entry) in enumerate(grouped.items()):
            raw = entry['raw']
            position = raw.get('position') or raw.get('center') or raw.get('bbox') or raw.get('bbox_xyxy') or raw
            values = [
                name,
                f"{entry['confidence']:.2f}",
                str(entry['count']),
                json.dumps(position, ensure_ascii=False)[:120],
            ]
            for col, value in enumerate(values):
                self.objects_table.setItem(row, col, QTableWidgetItem(value))
        self.add_cockpit_line(f'识别更新: {len(objects)} 个目标')
        self.touch_update()

    def on_system_status(self, msg: String) -> None:
        payload = self.parse_json_message(msg.data)
        names = ('bringup', 'mapping', 'navigation', 'zed', 'perception', 'llm')
        for row, name in enumerate(names):
            self.system_status_table.setItem(row, 0, QTableWidgetItem(name))
            self.system_status_table.setItem(row, 1, QTableWidgetItem(str(payload.get(name, 'unknown'))))
        line = (
            f"{time.strftime('%H:%M:%S')} {payload.get('last_command', '-')}: "
            f"{payload.get('message', '')}"
        )
        self.system_log.appendPlainText(line)
        self.touch_update()

    def add_timeline(self, text: str) -> None:
        self.timeline.insertItem(0, f'{time.strftime("%H:%M:%S")} {text}')
        while self.timeline.count() > 50:
            self.timeline.takeItem(self.timeline.count() - 1)
        self.add_cockpit_line(text)
        self.touch_update()

    def add_cockpit_line(self, text: str) -> None:
        self.cockpit_text.appendPlainText(f'{time.strftime("%H:%M:%S")} {text}')

    def show_error(self, text: str) -> None:
        self.add_timeline(f'错误: {text}')
        QMessageBox.warning(self, '提示', text)

    def parse_json_message(self, text: str) -> Dict[str, Any]:
        if not text:
            return {}
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            return {'message': text}
        return value if isinstance(value, dict) else {'message': text}

    def format_price(self, value: float) -> str:
        value = float(value)
        return str(int(value)) if value.is_integer() else f'{value:.2f}'

    def touch_update(self) -> None:
        self.last_update_ts = time.time()
        self.refresh_time_label()

    def refresh_time_label(self) -> None:
        if self.last_update_ts <= 0:
            self.time_label.setText('最近更新: -')
            return
        self.time_label.setText(f'最近更新: {time.strftime("%H:%M:%S", time.localtime(self.last_update_ts))}')

    def refresh_service_label(self) -> None:
        if self.bridge.b1_service_ready:
            self.b1_service_label.setText('B1服务: 就绪')
            self.b1_service_label.setStyleSheet('background: #079455; color: white;')
        else:
            self.b1_service_label.setText('B1服务: 未就绪')
            self.b1_service_label.setStyleSheet('background: #dc6803; color: white;')

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        if self.latest_task_image:
            self.show_task_image(self.latest_task_image)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    signals = UiSignals()
    bridge = RetailDisplayRosBridge(signals)
    if bridge.force_local_display:
        os.environ['DISPLAY'] = bridge.display or ':0'
    configure_input_method_environment()
    app = QApplication(sys.argv)
    window = RetailDisplayWindow(bridge, signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    if bridge.fullscreen:
        window.setWindowFlag(Qt.FramelessWindowHint, True)
        window.showFullScreen()
        screen = QApplication.primaryScreen()
        if screen is not None:
            window.setGeometry(screen.availableGeometry())
        window.fullscreen_button.setText('退出全屏')
    else:
        window.show()
        window.fullscreen_button.setText('进入全屏')

    try:
        exit_code = app.exec_()
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
