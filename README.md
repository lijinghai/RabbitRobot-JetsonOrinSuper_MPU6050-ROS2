# RabbitRobot-JetsonOrinSuper_MPU6050-ROS2
因为我使用的是 D435（非 D435i），它本身不带 IMU。在 SLAM 中，IMU 数据（加速度 + 角速度）可以帮助： 1. 提高位姿估计的稳定性 2. 实现视觉-惯性融合建图 3. 启动阶段更快收敛 —— IMU 可辅助初始化方向与加速度估计。 所以我用 MPU6050（通过 I²C 接 Jetson）来弥补 D435 缺少 IMU 的功能，这是一个非常合理的系统设计。
