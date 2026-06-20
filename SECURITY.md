# Security Policy

## Secrets

Do not commit API keys, access tokens, SSH keys, certificates, or local `.env` files.

The AI task layer reads DashScope credentials from the environment:

```bash
export DASHSCOPE_API_KEY=your_key_here
```

If a credential is accidentally committed, revoke it in the provider console first, then remove it from Git history before publishing again.

## Large Model Artifacts

Do not commit TensorRT engines, ONNX exports, PyTorch weights, or other model binaries. They are ignored by `.gitignore` and should be stored or transferred outside Git.

Expected local paths:

```text
src/ylhb_perception/models/yolo26.onnx
src/ylhb_perception/models/yolo26.engine
```

## Deployment

Run the robot with the least privilege practical for the hardware. Avoid broad permissions such as `chmod 777` on serial devices; prefer udev rules or dialout group membership.

Networked services such as DashScope should be enabled only when needed for the inspection task layer. Keep `enable_voice` and `enable_tts` disabled unless audio hardware and credentials are ready.

During builds, use the project build script or set `PYTHONNOUSERSITE=1` to prevent user-installed Python packages from shadowing ROS/Ubuntu build tooling.

## Reporting

For this inspection robot initialization framework, report issues through GitHub Issues in the repository. Include the ROS 2 distro, Jetson/JetPack version, launch command, logs, and reproduction steps.
