import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_perception')
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    default_params = os.path.join(pkg_dir, 'config', 'detector.yaml')
    default_model = os.path.join(
        workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')

    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    model_path = LaunchConfiguration('model_path')
    backend = LaunchConfiguration('backend')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    show_debug_window = LaunchConfiguration('show_debug_window')
    debug_window_max_hz = LaunchConfiguration('debug_window_max_hz')
    device = LaunchConfiguration('device')
    imgsz = LaunchConfiguration('imgsz')
    max_det = LaunchConfiguration('max_det')
    half = LaunchConfiguration('half')
    log_interval_sec = LaunchConfiguration('log_interval_sec')
    debug_image_max_hz = LaunchConfiguration('debug_image_max_hz')
    require_tensorrt = LaunchConfiguration('require_tensorrt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Perception parameter yaml file'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/rgb/color/rect/image',
            description='Input RGB image topic from ZED wrapper'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
            description='Input registered depth topic from ZED wrapper'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed/zed_node/rgb/color/rect/camera_info',
            description='Input camera info topic from ZED wrapper'),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='YOLO model path. Default is the Jetson-compiled TensorRT .engine model'),
        DeclareLaunchArgument(
            'backend',
            default_value='tensorrt',
            description='Inference backend name for logging and model loading'),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.35',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument(
            'iou_threshold',
            default_value='0.45',
            description='YOLO NMS IOU threshold'),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            description='Publish annotated debug image'),
        DeclareLaunchArgument(
            'show_debug_window',
            default_value='false',
            description='Show annotated detections in a local OpenCV window'),
        DeclareLaunchArgument(
            'debug_window_max_hz',
            default_value='15.0',
            description='Maximum local OpenCV debug window refresh rate'),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Inference device, such as cuda:0 or cpu'),
        DeclareLaunchArgument(
            'imgsz',
            default_value='960',
            description='YOLO inference image size. Must match the TensorRT engine input size'),
        DeclareLaunchArgument(
            'max_det',
            default_value='20',
            description='Maximum detections per frame'),
        DeclareLaunchArgument(
            'half',
            default_value='true',
            description='Use FP16 inference on CUDA when supported'),
        DeclareLaunchArgument(
            'log_interval_sec',
            default_value='2.0',
            description='Print FPS, inference latency, and detection summary every N seconds'),
        DeclareLaunchArgument(
            'debug_image_max_hz',
            default_value='5.0',
            description='Maximum debug image publish rate when publish_debug_image is true'),
        DeclareLaunchArgument(
            'require_tensorrt',
            default_value='true',
            description='Refuse to load non-TensorRT models for realtime inference'),

        Node(
            package='ylhb_perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'model_path': model_path,
                    'backend': backend,
                    'confidence_threshold': ParameterValue(confidence_threshold, value_type=float),
                    'iou_threshold': ParameterValue(iou_threshold, value_type=float),
                    'publish_debug_image': ParameterValue(publish_debug_image, value_type=bool),
                    'show_debug_window': ParameterValue(show_debug_window, value_type=bool),
                    'debug_window_max_hz': ParameterValue(debug_window_max_hz, value_type=float),
                    'device': device,
                    'imgsz': ParameterValue(imgsz, value_type=int),
                    'max_det': ParameterValue(max_det, value_type=int),
                    'half': ParameterValue(half, value_type=bool),
                    'log_interval_sec': ParameterValue(log_interval_sec, value_type=float),
                    'debug_image_max_hz': ParameterValue(debug_image_max_hz, value_type=float),
                    'require_tensorrt': ParameterValue(require_tensorrt, value_type=bool),
                },
            ],
        ),
        Node(
            package='ylhb_perception',
            executable='object_localizer_node.py',
            name='object_localizer_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'depth_topic': depth_topic,
                    'camera_info_topic': camera_info_topic,
                },
            ],
        ),
    ])
