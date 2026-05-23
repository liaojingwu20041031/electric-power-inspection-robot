import os
import errno
import glob
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_LIDAR_PORT = '/dev/robot_lidar'
DEFAULT_IMU_PORT = '/dev/robot_imu'
LIDAR_VENDOR_ID = '10c4'
LIDAR_PRODUCT_ID = 'ea60'
CH340_VENDOR_ID = '1a86'
CH340_PRODUCT_ID = '7523'


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


def _find_ch340_tty_ports():
    ports = []
    for path in sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyCH341USB*')):
        tty_name = os.path.basename(path)
        if _tty_usb_has_usb_id(tty_name, CH340_VENDOR_ID, CH340_PRODUCT_ID):
            ports.append(path)
    return ports


def _find_lidar_tty_ports():
    ports = []
    for path in sorted(glob.glob('/dev/ttyUSB*')):
        tty_name = os.path.basename(path)
        if _tty_usb_has_usb_id(tty_name, LIDAR_VENDOR_ID, LIDAR_PRODUCT_ID):
            ports.append(path)
    return ports


def _lsusb_has_ch340():
    try:
        output = subprocess.check_output(
            ['lsusb'], text=True, stderr=subprocess.DEVNULL
        ).lower()
    except (OSError, subprocess.CalledProcessError):
        return False
    return f'{CH340_VENDOR_ID}:{CH340_PRODUCT_ID}' in output


def _sysfs_has_usb_id(vendor_id, product_id):
    for device_path in glob.glob('/sys/bus/usb/devices/*'):
        if (
            _read_sysfs_text(os.path.join(device_path, 'idVendor')) == vendor_id
            and _read_sysfs_text(os.path.join(device_path, 'idProduct')) == product_id
        ):
            return True
    return False


def _has_ch340_usb_device():
    return _sysfs_has_usb_id(CH340_VENDOR_ID, CH340_PRODUCT_ID) or _lsusb_has_ch340()


def _ch340_interface_drivers():
    drivers = []
    for device_path in glob.glob('/sys/bus/usb/devices/*'):
        if (
            _read_sysfs_text(os.path.join(device_path, 'idVendor')) != CH340_VENDOR_ID
            or _read_sysfs_text(os.path.join(device_path, 'idProduct')) != CH340_PRODUCT_ID
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

    ch340_ttys = _find_ch340_tty_ports()
    if requested_port == DEFAULT_IMU_PORT and ch340_ttys:
        selected_port = ch340_ttys[0]
        ok, error = _can_open_serial(selected_port)
        if ok:
            return selected_port, (
                f'WARN: default IMU alias {DEFAULT_IMU_PORT} does not exist; '
                f'using detected CH340 tty {selected_port}. Run src/bind_usb.sh '
                'to create the stable /dev/robot_imu alias.'
            ), None
        return None, None, error

    if requested_port != DEFAULT_IMU_PORT:
        return None, None, (
            f'IMU requested port {requested_port} does not exist. '
            f'Pass imu_port:=/dev/ttyUSBx, imu_port:=/dev/ttyCH341USBx, '
            f'or use the default {DEFAULT_IMU_PORT}.'
        )

    if _has_ch340_usb_device():
        interface_drivers = _ch340_interface_drivers()
        if any(driver == 'usbfs' for _, driver in interface_drivers):
            driver_text = ', '.join(
                f'{interface}={driver}' for interface, driver in interface_drivers
            )
            return None, None, (
                f'CH340 is still claimed by usbfs ({driver_text}). '
                'Disable brltty/ModemManager and run sudo ./src/bind_usb.sh.'
            )
        return None, None, (
            'CH340 IMU USB device 1a86:7523 is enumerated, but no /dev/ttyUSB* '
            'or /dev/ttyCH341USB* was created for it. Run '
            'scripts/install_ch341_safe.sh --precheck and '
            'scripts/install_ch341_safe.sh --test-load.'
        )

    return None, None, (
        'IMU is required, but no CH340 IMU device was found. Plug in the '
        '1a86:7523 USB-TTL adapter or pass enable_imu:=false only for bench diagnostics.'
    )


def serial_nodes(context, *args, **kwargs):
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    imu_port = LaunchConfiguration('imu_port').perform(context)
    imu_baud_rate_text = LaunchConfiguration('imu_baud_rate').perform(context)
    lidar_baudrate_text = LaunchConfiguration('lidar_baudrate').perform(context)
    lidar_frame_id = LaunchConfiguration('lidar_frame_id').perform(context)
    enable_imu = LaunchConfiguration('enable_imu').perform(context).lower() in ('1', 'true', 'yes', 'on')

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
                f'ERROR: invalid imu_baud_rate={imu_baud_rate_text}; using 9600.'
            )))
            imu_baud_rate = 9600
        actions.append(Node(
            package='ylhb_base',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
            parameters=[
                {'serial_port': resolved_imu_port},
                {'baud_rate': imu_baud_rate}
            ]
        ))

    try:
        lidar_baudrate = int(lidar_baudrate_text)
    except ValueError:
        actions.append(LogInfo(msg=(
            f'ERROR: invalid lidar_baudrate={lidar_baudrate_text}; using 115200.'
        )))
        lidar_baudrate = 115200

    if os.path.exists(lidar_port):
        actions.append(Node(
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
        ))

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
        'imu_baud_rate', default_value='9600',
        description='Serial baudrate for the WIT IMU'
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

    # 获取动态的参数值
    base_port = LaunchConfiguration('base_port')
    base_backend = LaunchConfiguration('base_backend')

    use_zlac = IfCondition(PythonExpression(["'", base_backend, "' == 'zlac'"]))
    use_stm32 = IfCondition(PythonExpression(["'", base_backend, "' == 'stm32'"]))

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

    return LaunchDescription([
        base_backend_arg,
        base_port_arg,
        enable_imu_arg,
        imu_port_arg,
        imu_baud_rate_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        lidar_frame_id_arg,
        robot_state_publisher_node,
        zlac_base_node,
        stm32_base_node,
        OpaqueFunction(function=serial_nodes),
        ekf_node
    ])
