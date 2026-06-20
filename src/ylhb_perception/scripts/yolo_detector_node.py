#!/usr/bin/env python3

import json
import os
import time
from typing import Any, Dict, List

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('debug_image_topic', '/perception/debug_image')
        workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
        default_model = os.path.join(
            workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('backend', 'tensorrt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('half', True)
        self.declare_parameter('log_interval_sec', 2.0)
        self.declare_parameter('require_tensorrt', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.backend = self.get_parameter('backend').value
        self.actual_backend = self._detect_backend(self.model_path)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.device = self.get_parameter('device').value
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.half = bool(self.get_parameter('half').value)
        self.log_interval_sec = max(0.1, float(self.get_parameter('log_interval_sec').value))
        self.require_tensorrt = bool(self.get_parameter('require_tensorrt').value)
        self.frame_count = 0
        self.stats_start_time = time.monotonic()
        self.stats_last_log_time = self.stats_start_time
        self.stats_frame_count = 0
        self.stats_predict_ms_total = 0.0
        self.stats_trt_ms_total = 0.0
        self.stats_predict_count = 0

        self.bridge = CvBridge()
        self._warn_if_backend_mismatch()
        self.model = self._load_model()

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f"Jetson YOLO detector started: image={self.image_topic}, "
            f"detections={self.detections_topic}, debug_image={self.debug_image_topic}, "
            f"publish_debug_image={self.publish_debug_image}, "
            f"backend={self.actual_backend}, model={self.model_path}, "
            f"conf={self.confidence_threshold}, iou={self.iou_threshold}, imgsz={self.imgsz}, "
            f"device={self.device}, half={self.half}"
        )

    def _detect_backend(self, model_path: str) -> str:
        suffix = os.path.splitext(model_path)[1].lower()
        if suffix == '.engine':
            return 'tensorrt'
        if suffix == '.onnx':
            return 'onnxruntime'
        if suffix == '.pt':
            return 'pytorch'
        return self.backend

    def _warn_if_backend_mismatch(self) -> None:
        if self.backend == self.actual_backend:
            return
        self.get_logger().warn(
            f"Backend parameter '{self.backend}' does not match model file '{self.model_path}'. "
            f"Actual backend will be reported as '{self.actual_backend}'."
        )

    def _load_model(self) -> Any:
        if self.require_tensorrt and self.actual_backend != 'tensorrt':
            self.get_logger().error(
                f"Refusing to load '{self.model_path}' for realtime inference because it is "
                f"'{self.actual_backend}', not TensorRT. Use yolo26.engine, or set "
                "require_tensorrt:=false only for temporary debugging."
            )
            return None

        if not os.path.exists(self.model_path):
            self.get_logger().warn(
                f"Model file does not exist yet: {self.model_path}. "
                "Node will publish empty detections until a valid model path is configured."
            )
            return None

        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error(
                "Python package 'ultralytics' is not installed. "
                "Run scripts/install_jetson_dependencies.sh or install a TensorRT/YOLO backend."
            )
            return None

        try:
            return YOLO(self.model_path, task='detect')
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model '{self.model_path}': {exc}")
            return None

    def image_callback(self, msg: Image) -> None:
        callback_start = time.monotonic()
        self.frame_count += 1
        self.stats_frame_count += 1
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        detections: List[Dict[str, Any]] = []
        debug_image = image_bgr
        predict_ms = 0.0
        speed_ms = {'preprocess': 0.0, 'inference': 0.0, 'postprocess': 0.0}

        if self.model is not None:
            try:
                predict_start = time.monotonic()
                results = self.model.predict(
                    source=image_bgr,
                    conf=self.confidence_threshold,
                    iou=self.iou_threshold,
                    device=self.device,
                    imgsz=self.imgsz,
                    max_det=self.max_det,
                    half=self.half and str(self.device).startswith('cuda'),
                    verbose=False,
                )
                predict_ms = (time.monotonic() - predict_start) * 1000.0
                if results:
                    speed_ms.update(getattr(results[0], 'speed', {}) or {})
                self.stats_predict_ms_total += predict_ms
                self.stats_trt_ms_total += float(speed_ms.get('inference', 0.0))
                self.stats_predict_count += 1
                detections = self._parse_results(results)
                if self.publish_debug_image and results:
                    debug_image = results[0].plot()
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")

        payload = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'backend': self.backend,
            'actual_backend': self.actual_backend,
            'model_path': self.model_path,
            'detections': detections,
        }

        out = String()
        out.data = json.dumps(payload, ensure_ascii=True)
        self.detections_pub.publish(out)

        if self.publish_debug_image:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish debug image: {exc}")

        self._maybe_log_runtime_stats(
            msg=msg,
            detections=detections,
            predict_ms=predict_ms,
            speed_ms=speed_ms,
            callback_ms=(time.monotonic() - callback_start) * 1000.0,
        )

    def _maybe_log_runtime_stats(
        self,
        msg: Image,
        detections: List[Dict[str, Any]],
        predict_ms: float,
        speed_ms: Dict[str, float],
        callback_ms: float,
    ) -> None:
        now = time.monotonic()
        elapsed = now - self.stats_last_log_time
        if elapsed < self.log_interval_sec:
            return

        fps = self.stats_frame_count / elapsed if elapsed > 0.0 else 0.0
        avg_predict_ms = (
            self.stats_predict_ms_total / self.stats_predict_count
            if self.stats_predict_count
            else 0.0
        )
        avg_trt_ms = (
            self.stats_trt_ms_total / self.stats_predict_count
            if self.stats_predict_count
            else 0.0
        )
        summary = ', '.join(
            f"{d['class_name']}:{d['confidence']:.2f}" for d in detections[:5]
        )
        if not summary:
            summary = 'no detections'

        self.get_logger().info(
            f"帧率={fps:.1f} FPS, 帧号={self.frame_count}, 图像={msg.width}x{msg.height}, "
            f"目标={len(detections)} ({summary}), "
            f"总预测={predict_ms:.1f}ms, 平均预测={avg_predict_ms:.1f}ms, "
            f"TRT={float(speed_ms.get('inference', 0.0)):.1f}ms, 平均TRT={avg_trt_ms:.1f}ms, "
            f"预处理={float(speed_ms.get('preprocess', 0.0)):.1f}ms, "
            f"后处理={float(speed_ms.get('postprocess', 0.0)):.1f}ms, "
            f"总耗时={callback_ms:.1f}ms, 后端={self.actual_backend}"
        )

        self.stats_last_log_time = now
        self.stats_frame_count = 0
        self.stats_predict_ms_total = 0.0
        self.stats_trt_ms_total = 0.0
        self.stats_predict_count = 0

    def _parse_results(self, results: Any) -> List[Dict[str, Any]]:
        if not results:
            return []

        result = results[0]
        names = getattr(result, 'names', {}) or {}
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return []

        detections: List[Dict[str, Any]] = []
        for box in boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy().tolist()
            conf = float(box.conf[0].detach().cpu().item()) if box.conf is not None else 0.0
            cls_id = int(box.cls[0].detach().cpu().item()) if box.cls is not None else -1
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            detections.append({
                'class_id': cls_id,
                'class_name': str(names.get(cls_id, cls_id)),
                'confidence': conf,
                'bbox_xyxy': [x1, y1, x2, y2],
                'bbox_center': [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
                'bbox_size': [max(0.0, x2 - x1), max(0.0, y2 - y1)],
            })
        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
