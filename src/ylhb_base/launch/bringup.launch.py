import os
import errno
import glob
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_LIDAR_PORT = '/dev/robot_lidar'
DEFAULT_IMU_PORT = '/dev/robot_imu'
LIDAR_VENDOR_ID = '10c4'
LIDAR_PRODUCT_ID = 'ea60'
N300_VENDOR_ID = '1a86'
N300_PRODUCT_ID = '55d4'


def critical_exit_handler(node, name):
    def on_exit(event, _context):
        if event.returncode in (0, -signal.SIGINT, -signal.SIGTERM):
            return []
        return [
            LogInfo(msg=f'ERROR: critical bringup node {name} exited; shutting down bringup.'),
            EmitEvent(event=Shutdown(reason=f'critical bringup node {name} exited')),
        ]

    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=on_exit,
        )
    )


def _read_sysfs_text(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return f.read().strip().lower()
    except OSError:
        return ''


def _tty_usb_has_usb_id(tty_name, vendor_id, product_id):
    device_path = os.path.realpath(f'/sys/class/tty/{tty_name}/device')
    current = device_path
    while current and current != '/':
        if (
            _read_sysfs_text(os.path.join(current, 'idVendor')) == vendor_id
            and _read_sysfs_text(os.path.join(current, 'idProduct')) == product_id
        ):
            return True
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return False


def _find_n300_tty_ports():
    ports = []
    candidates = (
        glob.glob('/dev/ttyACM*')
        + glob.glob('/dev/ttyCH343USB*')
        + glob.glob('/dev/ttyUSB*')
    )
    for path in sorted(candidates):
        tty_name = os.path.basename(path)
        if _tty_usb_has_usb_id(tty_name, N300_VENDOR_ID, N300_PRODUCT_ID):
            ports.append(path)
    return ports


def _find_lidar_tty_ports():
    ports = []
    for path in sorted(glob.glob('/dev/ttyUSB*')):
        tty_name = os.path.basename(path)
        if _tty_usb_has_usb_id(tty_name, LIDAR_VENDOR_ID, LIDAR_PRODUCT_ID):
            ports.append(path)
    return ports


def _lsusb_has_n300():
    try:
        output = subprocess.check_output(
            ['lsusb'], text=True, stderr=subprocess.DEVNULL
        ).lower()
    except (OSError, subprocess.CalledProcessError):
        return False
    return f'{N300_VENDOR_ID}:{N300_PRODUCT_ID}' in output


def _sysfs_has_usb_id(vendor_id, product_id):
    for device_path in glob.glob('/sys/bus/usb/devices/*'):
        if (
            _read_sysfs_text(os.path.join(device_path, 'idVendor')) == vendor_id
            and _read_sysfs_text(os.path.join(device_path, 'idProduct')) == product_id
        ):
            return True
    return False


def _has_n300_usb_device():
    return _sysfs_has_usb_id(N300_VENDOR_ID, N300_PRODUCT_ID) or _lsusb_has_n300()


def _n300_interface_drivers():
    drivers = []
    for device_path in glob.glob('/sys/bus/usb/devices/*'):
        if (
            _read_sysfs_text(os.path.join(device_path, 'idVendor')) != N300_VENDOR_ID
            or _read_sysfs_text(os.path.join(device_path, 'idProduct')) != N300_PRODUCT_ID
        ):
            continue

        for interface_path in glob.glob(f'{device_path}:*'):
            if not os.path.exists(os.path.join(interface_path, 'bInterfaceNumber')):
                continue
            driver_path = os.path.join(interface_path, 'driver')
            if os.path.islink(driver_path):
                driver = os.path.basename(os.path.realpath(driver_path))
            else:
                driver = 'none'
            drivers.append((os.path.basename(interface_path), driver))
    return drivers


def _can_open_serial(path):
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        return True, ''
    except OSError as exc:
        if exc.errno in (errno.EACCES, errno.EPERM):
            return False, (
                f'IMU serial port {path} exists but permission is denied. '
                'Run src/bind_usb.sh or add the user to the dialout group, then replug the IMU.'
            )
        if exc.errno == errno.ENOENT:
            return False, f'IMU serial port {path} does not exist.'
        return False, f'IMU serial port {path} cannot be opened: {exc.strerror}.'


def _resolve_required_imu_port(requested_port):
    if os.path.exists(requested_port):
        ok, error = _can_open_serial(requested_port)
        if ok:
            return requested_port, None, None
        return None, None, error

    n300_ttys = _find_n300_tty_ports()
    if requested_port == DEFAULT_IMU_PORT and n300_ttys:
        selected_port = n300_ttys[0]
        ok, error = _can_open_serial(selected_port)
        if ok:
            return selected_port, (
                f'WARN: default IMU alias {DEFAULT_IMU_PORT} does not exist; '
                f'using detected N300WP PRO tty {selected_port}. Run src/bind_usb.sh '
                'to create the stable /dev/robot_imu alias.'
            ), None
        return None, None, error

    if requested_port != DEFAULT_IMU_PORT:
        return None, None, (
            f'IMU requested port {requested_port} does not exist. '
            f'Pass imu_port:=/dev/ttyACMx, imu_port:=/dev/ttyCH343USBx, '
            f'or use the default {DEFAULT_IMU_PORT}.'
        )

    if _has_n300_usb_device():
        interface_drivers = _n300_interface_drivers()
        if any(driver == 'usbfs' for _, driver in interface_drivers):
            driver_text = ', '.join(
                f'{interface}={driver}' for interface, driver in interface_drivers
            )
            return None, None, (
                f'N300WP PRO CH9102 is still claimed by usbfs ({driver_text}). '
                'Disable ModemManager and run sudo ./src/bind_usb.sh.'
            )
        return None, None, (
            'N300WP PRO USB device 1a86:55d4 is enumerated, but no /dev/ttyACM*, '
            '/dev/ttyCH343USB*, or /dev/ttyUSB* device was created. Check the '
            'cdc_acm/CH343 driver and reconnect the IMU.'
        )

    return None, None, (
        'IMU is required, but no N300WP PRO device was found. Plug in the '
        '1a86:55d4 CH9102 device or pass enable_imu:=false only for bench diagnostics.'
    )


def serial_nodes(context, *args, **kwargs):
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    imu_port = LaunchConfiguration('imu_port').perform(context)
    imu_baud_rate_text = LaunchConfiguration('imu_baud_rate').perform(context)
    imu_frame_id = LaunchConfiguration('imu_frame_id').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    lidar_baudrate_text = LaunchConfiguration('lidar_baudrate').perform(context)
    lidar_frame_id = LaunchConfiguration('lidar_frame_id').perform(context)
    enable_imu = (
        LaunchConfiguration('enable_imu').perform(context).lower()
        in ('1', 'true', 'yes', 'on')
    )

    actions = []
    if not os.path.exists(lidar_port):
        lidar_ttys = _find_lidar_tty_ports()
        if lidar_port == DEFAULT_LIDAR_PORT and lidar_ttys:
            actions.append(LogInfo(msg=(
                f'WARN: default LiDAR alias {lidar_port} does not exist; '
                f'using detected CP2102 tty {lidar_ttys[0]}. Run src/bind_usb.sh '
                'to create the stable /dev/robot_lidar alias.'
            )))
            lidar_port = lidar_ttys[0]
        else:
            actions.append(LogInfo(msg=(
                f'ERROR: LiDAR serial port {lidar_port} does not exist. '
                'Check /dev/robot_lidar or pass lidar_port:=/dev/ttyUSBx.'
            )))

    if enable_imu:
        resolved_imu_port, imu_warning, imu_error = _resolve_required_imu_port(imu_port)
        if imu_error:
            raise RuntimeError(f'IMU required but unavailable: {imu_error}')
        if imu_warning:
            actions.append(LogInfo(msg=imu_warning))
        try:
            imu_baud_rate = int(imu_baud_rate_text)
        except ValueError:
            actions.append(LogInfo(msg=(
                f'ERROR: invalid imu_baud_rate={imu_baud_rate_text}; using 115200.'
            )))
            imu_baud_rate = 115200
        imu_node = Node(
            package='hipnuc_imu',
            executable='talker',
            name='IMU_publisher',
            output='screen',
            parameters=[
                {'serial_port': resolved_imu_port},
                {'baud_rate': imu_baud_rate},
                {'frame_id': imu_frame_id},
                {'imu_topic': imu_topic}
            ]
        )
        actions.extend((imu_node, critical_exit_handler(imu_node, 'IMU_publisher')))

    try:
        lidar_baudrate = int(lidar_baudrate_text)
    except ValueError:
        actions.append(LogInfo(msg=(
            f'ERROR: invalid lidar_baudrate={lidar_baudrate_text}; using 115200.'
        )))
        lidar_baudrate = 115200

    if os.path.exists(lidar_port):
        lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baudrate,
                'frame_id': lidar_frame_id,
                'inverted': False,
                'angle_compensate': True
            }]
        )
        actions.extend((lidar_node, critical_exit_handler(lidar_node, 'rplidar_node')))

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_base')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    base_kinematics_path = os.path.join(pkg_dir, 'config', 'base_kinematics.yaml')
    zlac_config_path = os.path.join(pkg_dir, 'config', 'zlac8015d.yaml')

    # 引入机器人模型的 urdf.xacro 文件定位
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ylhb.urdf.xacro')

    # 声明变量作为启动参数（Launch Arguments），方便在命令行动态修改串口号
    base_port_arg = DeclareLaunchArgument(
        'base_port', default_value='/dev/ttyS1',
        description='Serial port for the STM32 base controller fallback'
    )
    base_backend_arg = DeclareLaunchArgument(
        'base_backend', default_value='zlac',
        description='Chassis backend: zlac or stm32'
    )
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu', default_value='true',
        description='Enable required IMU driver node'
    )
    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value=DEFAULT_IMU_PORT,
        description='Serial port for the IMU sensor'
    )
    imu_baud_rate_arg = DeclareLaunchArgument(
        'imu_baud_rate', default_value='115200',
        description='Serial baudrate for the N300WP PRO IMU'
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id', default_value='imu_link',
        description='Frame id for N300WP PRO IMU messages'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/data',
        description='ROS sensor_msgs/Imu topic published by N300WP PRO'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value=DEFAULT_LIDAR_PORT,
        description='Serial port for the LiDAR'
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='Serial baudrate for RPLidar A2M8'
    )
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='laser_link',
        description='Frame id for RPLidar laser scans'
    )
    enable_rtk_arg = DeclareLaunchArgument(
        'enable_rtk', default_value='false',
        description='Enable WTRTK980 NMEA RTK receiver'
    )
    rtk_port_arg = DeclareLaunchArgument(
        'rtk_port', default_value='/dev/rtk_4g',
        description='Serial port for the WTRTK980 RTK receiver'
    )
    rtk_baud_arg = DeclareLaunchArgument(
        'rtk_baud', default_value='115200',
        description='Serial baudrate for the WTRTK980 RTK receiver'
    )
    rtk_frame_id_arg = DeclareLaunchArgument(
        'rtk_frame_id', default_value='gps_link',
        description='Frame id for RTK messages'
    )

    # 获取动态的参数值
    base_port = LaunchConfiguration('base_port')
    base_backend = LaunchConfiguration('base_backend')

    use_zlac = IfCondition(PythonExpression(["'", base_backend, "' == 'zlac'"]))
    use_stm32 = IfCondition(PythonExpression(["'", base_backend, "' == 'stm32'"]))
    enable_rtk = IfCondition(LaunchConfiguration('enable_rtk'))

    # 默认 ZLAC8015D SocketCAN 底盘后端，关闭自身 TF，让 EKF 接管
    zlac_base_node = Node(
        package='ylhb_base',
        executable='zlac8015d_canopen_controller',
        name='zlac8015d_canopen_controller',
        output='screen',
        condition=use_zlac,
        parameters=[
            base_kinematics_path,
            zlac_config_path,
            {'publish_tf': False}
        ]
    )

    # STM32 串口底盘控制节点作为回退方案
    stm32_base_node = Node(
        package='ylhb_base',
        executable='base_controller',
        name='base_controller',
        output='screen',
        condition=use_stm32,
        parameters=[
            {'serial_port': base_port},
            {'publish_tf': False}  # 重要：防止 TF 冲突
        ]
    )

    # 机器人状态发布节点 (统一处理和发布机器人的全套物理 TF 关系)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    # Robot Localization EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    rtk_node = Node(
        package='ylhb_base',
        executable='wtrtk980_nmea_node',
        name='wtrtk980_nmea_node',
        output='screen',
        condition=enable_rtk,
        parameters=[{
            'port': LaunchConfiguration('rtk_port'),
            'baud': ParameterValue(LaunchConfiguration('rtk_baud'), value_type=int),
            'frame_id': LaunchConfiguration('rtk_frame_id'),
        }]
    )

    return LaunchDescription([
        base_backend_arg,
        base_port_arg,
        enable_imu_arg,
        imu_port_arg,
        imu_baud_rate_arg,
        imu_frame_id_arg,
        imu_topic_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        lidar_frame_id_arg,
        enable_rtk_arg,
        rtk_port_arg,
        rtk_baud_arg,
        rtk_frame_id_arg,
        OpaqueFunction(function=serial_nodes),
        robot_state_publisher_node,
        zlac_base_node,
        stm32_base_node,
        ekf_node,
        rtk_node,
        critical_exit_handler(robot_state_publisher_node, 'robot_state_publisher'),
        critical_exit_handler(zlac_base_node, 'zlac8015d_canopen_controller'),
        critical_exit_handler(stm32_base_node, 'base_controller'),
        critical_exit_handler(ekf_node, 'ekf_filter_node'),
    ])
