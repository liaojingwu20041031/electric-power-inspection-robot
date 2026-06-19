import json
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel,
    QLineEdit, QPushButton, QPlainTextEdit, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ylhb_interfaces.msg import SayText, TaskEvent, TaskStatus, VoiceStatus


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class UiSignals(QObject):
    system_status = pyqtSignal(dict)
    task_context = pyqtSignal(dict)
    task_event = pyqtSignal(object)
    task_status = pyqtSignal(object)
    say_text = pyqtSignal(object)
    voice_status = pyqtSignal(object)
    localized_objects = pyqtSignal(str)


class InspectionDisplayRosBridge(Node):
    def __init__(self, signals: UiSignals) -> None:
        super().__init__('inspection_display_ui_node')
        self.signals = signals
        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('task_event_topic', '/inspection_ai/task_event')
        self.declare_parameter('task_status_topic', '/inspection_ai/task_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('task_context_status_topic', '/inspection_ai/task_context_status')
        self.declare_parameter('voice_status_topic', '/inspection_ai/voice_status')
        self.declare_parameter('start_voice_session_service_name', '/inspection_ai/start_voice_session')
        self.declare_parameter('stop_voice_session_service_name', '/inspection_ai/stop_voice_session')
        self.declare_parameter('capture_voice_service_name', '/inspection_ai/capture_voice')
        self.declare_parameter('localized_objects_topic', '/perception/localized_objects')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_system_mode', 'ready')
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('display', ':0')
        self.declare_parameter('force_local_display', True)

        self.text_pub = self.create_publisher(String, self.get_parameter('text_command_topic').value, 10)
        self.system_mode_pub = self.create_publisher(String, self.get_parameter('system_mode_topic').value, latched_qos())
        self.system_command_pub = self.create_publisher(String, self.get_parameter('system_command_topic').value, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(String, self.get_parameter('system_status_topic').value, self.system_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('task_context_status_topic').value, self.task_context_callback, latched_qos())
        self.create_subscription(TaskEvent, self.get_parameter('task_event_topic').value, self.task_event_callback, 10)
        self.create_subscription(TaskStatus, self.get_parameter('task_status_topic').value, self.task_status_callback, 10)
        self.create_subscription(SayText, self.get_parameter('say_text_topic').value, self.say_text_callback, 10)
        self.create_subscription(VoiceStatus, self.get_parameter('voice_status_topic').value, self.voice_status_callback, 10)
        self.create_subscription(String, self.get_parameter('localized_objects_topic').value, self.localized_objects_callback, 10)
        self.start_voice_client = self.create_client(Trigger, self.get_parameter('start_voice_session_service_name').value)
        self.stop_voice_client = self.create_client(Trigger, self.get_parameter('stop_voice_session_service_name').value)
        self.capture_voice_client = self.create_client(Trigger, self.get_parameter('capture_voice_service_name').value)
        self.publish_system_mode(str(self.get_parameter('initial_system_mode').value))
        self.get_logger().info('Inspection display UI bridge started.')

    def publish_text_command(self, text: str, source: str = 'ui') -> None:
        msg = String()
        msg.data = json.dumps({'schema_version': '1.0', 'source': source, 'text': text, 'timestamp': time.time()}, ensure_ascii=False)
        self.text_pub.publish(msg)

    def publish_system_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.system_mode_pub.publish(msg)

    def publish_system_command(self, command: str, **extra: Any) -> None:
        payload = {'schema_version': '1.0', 'command': command, 'source': 'ui', 'timestamp': time.time()}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.system_command_pub.publish(msg)

    def publish_twist(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def call_voice_service(self, name: str) -> None:
        client = {'start': self.start_voice_client, 'stop': self.stop_voice_client, 'capture': self.capture_voice_client}[name]
        if client.service_is_ready():
            client.call_async(Trigger.Request())

    def system_status_callback(self, msg: String) -> None:
        self.signals.system_status.emit(self.parse_json(msg.data))

    def task_context_callback(self, msg: String) -> None:
        self.signals.task_context.emit(self.parse_json(msg.data))

    def task_event_callback(self, msg: TaskEvent) -> None:
        self.signals.task_event.emit(msg)

    def task_status_callback(self, msg: TaskStatus) -> None:
        self.signals.task_status.emit(msg)

    def say_text_callback(self, msg: SayText) -> None:
        self.signals.say_text.emit(msg)

    def voice_status_callback(self, msg: VoiceStatus) -> None:
        self.signals.voice_status.emit(msg)

    def localized_objects_callback(self, msg: String) -> None:
        self.signals.localized_objects.emit(msg.data)

    def parse_json(self, text: str) -> Dict[str, Any]:
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {'value': value}
        except json.JSONDecodeError:
            return {'raw': text}


class InspectionDisplayWindow(QWidget):
    def __init__(self, bridge: InspectionDisplayRosBridge, signals: UiSignals) -> None:
        super().__init__()
        self.bridge = bridge
        self.signals = signals
        self.setWindowTitle('电力巡检机器人总控台')
        self.resize(1180, 760)
        self.system_status_table = QTableWidget(0, 2)
        self.context_table = QTableWidget(0, 2)
        self.timeline = QPlainTextEdit()
        self.timeline.setReadOnly(True)
        self.command_input = QLineEdit()
        self.localized_view = QPlainTextEdit()
        self.localized_view.setReadOnly(True)
        self.voice_label = QLabel('语音: -')
        self.build_ui()
        self.connect_signals()
        if bool(self.bridge.get_parameter('fullscreen').value):
            self.showFullScreen()

    def build_ui(self) -> None:
        root = QVBoxLayout(self)
        title = QLabel('电力巡检机器人控制台')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 24px; font-weight: 700; padding: 8px;')
        root.addWidget(title)
        top = QHBoxLayout()
        top.addWidget(self.build_system_panel(), 2)
        top.addWidget(self.build_command_panel(), 3)
        root.addLayout(top, 2)
        lower = QHBoxLayout()
        lower.addWidget(self.build_context_panel(), 2)
        lower.addWidget(self.build_log_panel(), 3)
        root.addLayout(lower, 3)
        self.setStyleSheet('QGroupBox { font-weight: 600; } QPushButton { min-height: 30px; }')

    def build_system_panel(self) -> QGroupBox:
        box = QGroupBox('系统控制')
        layout = QVBoxLayout(box)
        grid = QGridLayout()
        buttons = [
            ('进入准备', lambda: self.set_mode('ready')),
            ('启动巡检节点', lambda: self.system_command('start_robot_stack')),
            ('停止巡检节点', lambda: self.system_command('stop_robot_stack')),
            ('启动底盘/雷达', lambda: self.system_command('start_bringup')),
            ('停止底盘/雷达', lambda: self.system_command('stop_bringup')),
            ('启动导航', lambda: self.system_command('start_navigation')),
            ('停止导航', lambda: self.system_command('stop_navigation')),
            ('启动 ZED', lambda: self.system_command('start_zed')),
            ('停止 ZED', lambda: self.system_command('stop_zed')),
            ('启动感知', lambda: self.system_command('start_perception')),
            ('停止感知', lambda: self.system_command('stop_perception')),
            ('软件急停', lambda: self.system_command('emergency_stop')),
        ]
        for i, (label, callback) in enumerate(buttons):
            button = QPushButton(label)
            button.clicked.connect(callback)
            grid.addWidget(button, i // 2, i % 2)
        layout.addLayout(grid)
        save_button = QPushButton('保存地图')
        save_button.clicked.connect(self.save_map)
        layout.addWidget(save_button)
        self.system_status_table.setHorizontalHeaderLabels(['模块', '状态'])
        layout.addWidget(self.system_status_table)
        return box

    def build_command_panel(self) -> QGroupBox:
        box = QGroupBox('巡检任务指令')
        layout = QVBoxLayout(box)
        self.command_input.setPlaceholderText('例如：开始巡检任务 / 暂停巡检 / 检查 1 号开关柜 / 人工接管')
        layout.addWidget(self.command_input)
        row = QHBoxLayout()
        for label, callback in (
            ('发送指令', self.send_command),
            ('演示巡检任务', lambda: self.bridge.publish_text_command('开始巡检任务')),
            ('开启语音', lambda: self.bridge.call_voice_service('start')),
            ('关闭语音', lambda: self.bridge.call_voice_service('stop')),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        teleop = QGridLayout()
        for i, (label, linear, angular) in enumerate((('前进', 0.12, 0.0), ('后退', -0.12, 0.0), ('左转', 0.0, 0.45), ('右转', 0.0, -0.45), ('停止', 0.0, 0.0))):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, l=linear, a=angular: self.bridge.publish_twist(l, a))
            teleop.addWidget(button, i // 3, i % 3)
        layout.addLayout(teleop)
        layout.addWidget(self.voice_label)
        self.localized_view.setPlaceholderText('感知节点输出会显示在这里。')
        layout.addWidget(self.localized_view)
        return box

    def build_context_panel(self) -> QGroupBox:
        box = QGroupBox('任务上下文')
        layout = QVBoxLayout(box)
        self.context_table.setHorizontalHeaderLabels(['字段', '值'])
        layout.addWidget(self.context_table)
        return box

    def build_log_panel(self) -> QGroupBox:
        box = QGroupBox('事件时间线')
        layout = QVBoxLayout(box)
        layout.addWidget(self.timeline)
        return box

    def connect_signals(self) -> None:
        self.signals.system_status.connect(self.update_system_status)
        self.signals.task_context.connect(self.update_context)
        self.signals.task_event.connect(self.on_task_event)
        self.signals.task_status.connect(self.on_task_status)
        self.signals.say_text.connect(self.on_say_text)
        self.signals.voice_status.connect(self.on_voice_status)
        self.signals.localized_objects.connect(self.on_localized_objects)

    def send_command(self) -> None:
        text = self.command_input.text().strip()
        if not text:
            return
        self.bridge.publish_text_command(text)
        self.add_log(f'UI 指令: {text}')
        self.command_input.clear()

    def set_mode(self, mode: str) -> None:
        self.bridge.publish_system_mode(mode)
        self.add_log(f'系统模式: {mode}')

    def system_command(self, command: str) -> None:
        self.bridge.publish_system_command(command)
        self.add_log(f'系统命令: {command}')

    def save_map(self) -> None:
        default_name = time.strftime('inspection_map_%Y%m%d_%H%M')
        name, ok = QInputDialog.getText(self, '保存地图', '请输入地图名称：', text=default_name)
        if ok and name.strip():
            self.bridge.publish_system_command('save_map', map_name=name.strip())
            self.add_log(f'保存地图: {name.strip()}')

    def update_system_status(self, payload: Dict[str, Any]) -> None:
        rows = [(k, v) for k, v in payload.items() if k not in ('schema_version', 'timestamp')]
        self.system_status_table.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.system_status_table.setItem(row, 0, QTableWidgetItem(str(key)))
            self.system_status_table.setItem(row, 1, QTableWidgetItem(str(value)))

    def update_context(self, payload: Dict[str, Any]) -> None:
        rows = [(k, v) for k, v in payload.items() if k not in ('schema_version', 'timestamp')]
        self.context_table.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.context_table.setItem(row, 0, QTableWidgetItem(str(key)))
            self.context_table.setItem(row, 1, QTableWidgetItem(self.format_value(value)))

    def on_task_event(self, msg: TaskEvent) -> None:
        self.add_log(f'任务事件: {msg.intent} task={msg.task_id} target={msg.item_name or msg.item_id}')

    def on_task_status(self, msg: TaskStatus) -> None:
        self.add_log(f'任务状态: {msg.task_id} {msg.stage}/{msg.status} {msg.reason}')

    def on_say_text(self, msg: SayText) -> None:
        self.add_log(f'播报: {msg.text}')

    def on_voice_status(self, msg: VoiceStatus) -> None:
        self.voice_label.setText(f'语音: {msg.state} {msg.text}')

    def on_localized_objects(self, text: str) -> None:
        self.localized_view.setPlainText(text[:4000])

    def add_log(self, text: str) -> None:
        self.timeline.appendPlainText(f'[{time.strftime("%H:%M:%S")}] {text}')

    def format_value(self, value: Any) -> str:
        if isinstance(value, (dict, list)):
            return json.dumps(value, ensure_ascii=False)
        return str(value)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    app = QApplication([])
    signals = UiSignals()
    bridge = InspectionDisplayRosBridge(signals)
    window = InspectionDisplayWindow(bridge, signals)
    window.show()
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(bridge, timeout_sec=0.0))
    timer.start(20)
    try:
        app.exec_()
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
