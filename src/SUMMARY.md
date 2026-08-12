# 目录

[前言：这本书能带你到哪里](./preface.md)
[如何使用本书](./how-to-use.md)

# 第一部分 · 认识智能驾驶

- [1.1 什么是自动驾驶：从 L0 到 L5](./part1/levels.md)
- [1.2 一辆自动驾驶车的软件全景](./part1/architecture.md)
- [1.3 感知—定位—预测—规划—控制 五大模块](./part1/pipeline.md)
- [1.4 为什么在智能驾驶里用 Rust](./part1/why-rust.md)
- [1.5 行业现状、岗位与技术栈地图](./part1/industry.md)

# 第二部分 · 面向智驾的 Rust 语言基础

- [2.1 搭建 Rust 开发环境](./part2/setup.md)
- [2.2 所有权、借用与生命周期](./part2/ownership.md)
- [2.3 类型系统、trait 与泛型](./part2/traits.md)
- [2.4 错误处理：Result、Option 与 anyhow](./part2/errors.md)
- [2.5 并发与异步：线程、channel、tokio](./part2/concurrency.md)
- [2.6 智驾常用 crate 生态速览](./part2/ecosystem.md)
- [2.7 性能工程、unsafe 与 C/C++ FFI](./part2/performance-ffi.md)

# 第三部分 · 传感器、坐标系与数据

- [3.1 传感器全家福：相机、激光雷达、毫米波、IMU、GNSS](./part3/sensors.md)
- [3.2 坐标系与刚体变换](./part3/coordinates.md)
- [3.3 时间戳、同步与数据流](./part3/time-sync.md)
- [3.4 用 Rust 表达传感器数据](./part3/data-model.md)
- [3.5 传感器标定、运动补偿与在线健康监测](./part3/calibration.md)

# 第四部分 · 感知

- [4.1 感知任务总览](./part4/overview.md)
- [4.2 图像处理基础与 Rust 实现](./part4/image.md)
- [4.3 深度学习推理：onnxruntime / tract / candle](./part4/inference.md)
- [4.4 点云处理与激光雷达感知](./part4/pointcloud.md)
- [4.5 多目标跟踪与传感器融合](./part4/tracking-fusion.md)
- [4.6 感知数据闭环与量产评测](./part4/data-loop.md)

# 第五部分 · 定位、建图与状态估计

- [5.1 状态估计与卡尔曼滤波](./part5/kalman.md)
- [5.2 定位：GNSS/IMU 组合导航与地图匹配](./part5/localization.md)
- [5.3 SLAM 与高精地图概览](./part5/slam-hdmap.md)

# 第六部分 · 预测与规划

- [6.1 行为预测](./part6/prediction.md)
- [6.2 路径与轨迹规划](./part6/planning.md)
- [6.3 决策与行为规划](./part6/decision.md)

# 第七部分 · 控制

- [7.1 车辆运动学与动力学模型](./part7/vehicle-model.md)
- [7.2 横向与纵向控制：PID 到 MPC](./part7/control.md)

# 第八部分 · 中间件与系统集成

- [8.1 自动驾驶中间件全景](./part8/middleware.md)
- [8.2 ROS 2 与 rclrs：用 Rust 写节点](./part8/ros2-rclrs.md)
- [8.3 Zenoh、DDS 与零拷贝通信](./part8/zenoh-dds.md)
- [8.4 消息定义与序列化](./part8/messages.md)
- [8.5 运行时工程：背压、可观测性与资源预算](./part8/runtime-engineering.md)

# 第九部分 · 仿真、测试与安全

- [9.1 仿真：CARLA 与数据回放](./part9/simulation.md)
- [9.2 测试策略：单元、集成与录制回放](./part9/testing.md)
- [9.3 功能安全、实时性与部署](./part9/safety.md)
- [9.4 汽车网络安全、供应链与安全 OTA](./part9/cybersecurity.md)
- [9.5 系统工程、安全论证与故障注入](./part9/safety-case.md)

# 第十部分 · 实战与职业

- [10.1 实战项目：搭一个最小自动驾驶栈](./part10/project.md)
- [10.2 学习路线图与求职准备](./part10/career.md)
- [10.3 常见术语表](./part10/glossary.md)
- [10.4 逐章能力验收与作品集证据](./part10/competency.md)
- [10.5 中高级毕业项目：从最小栈到可评审系统](./part10/capstone.md)

---

[后记与延伸资源](./afterword.md)
