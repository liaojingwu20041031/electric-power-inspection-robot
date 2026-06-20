# YOLO26 模型目录

最终部署流程：

```text
PC 训练 YOLO -> PC 导出 yolo26.onnx -> 复制到 Jetson -> Jetson 编译 yolo26.engine -> Jetson 运行 engine
```

推荐文件：

```text
yolo26.pt       # 原始训练权重，留给 PC 端训练/导出
yolo26.onnx     # PC 端导出的跨平台中间模型
yolo26.engine   # Jetson 本机编译出的 TensorRT FP16 推理引擎，实时检测默认使用
```

Jetson 端一次性编译 TensorRT engine：

```bash
cd ~/ros2_DL
source install/setup.bash
ros2 run ylhb_perception export_yolo_trt.py \
  --onnx /home/nvidia/ros2_DL/src/ylhb_perception/models/yolo26.onnx \
  --output /home/nvidia/ros2_DL/src/ylhb_perception/models/yolo26.engine \
  --workspace 2048
```

Jetson 实时推理：

```bash
ros2 launch ylhb_perception perception.launch.py \
  model_path:=/home/nvidia/ros2_DL/src/ylhb_perception/models/yolo26.engine \
  backend:=tensorrt \
  imgsz:=960 \
  half:=true \
  device:=cuda:0
```
