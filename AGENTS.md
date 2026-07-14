# Agent Preferences

- 默认使用中文回复用户，除非用户明确要求英文或其他语言。
- 修改本仓库代码或测试前，遵守 `docs/好测试规范.md`。
- 需要新增、调整或审查测试时，使用 `skills/good-testing/SKILL.md`。
- 处理 Nav2、AMCL、TF、lifecycle、巡逻启动、传感器、底盘、CAN、串口、相机、SLAM 或 Jetson 实机问题时，必须使用 `skills/ros2-practice-first/SKILL.md`。
- 处理实机路线、巡逻、Nav2 目标、遥控或任何可能让机器人运动的操作时，必须使用 `skills/robot-motion-test-boundary/SKILL.md`。实地跑路线和实际运动由用户现场执行；Agent 仅可做该技能允许的只读或明确无执行器副作用的话题通信检查。
- ROS2 运行故障默认不新增测试文件，不运行全仓库测试；先依据真实日志和提交差异做最小修复，再给出实机验收步骤。
- Mock/Fake、静态字符串断言和纯模拟启动流程不能作为实机功能完成证明。
