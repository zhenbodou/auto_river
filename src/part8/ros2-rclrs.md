# 8.2 ROS 2 与 rclrs：用 Rust 写节点

这一节先把 ROS 2 的架构讲清（节点、话题、消息、DDS、QoS），再用 Rust 客户端 `rclrs` 展示一对节点——一个发布 `Obstacle` 消息，一个订阅并处理。示例表达完整结构；实际可编译性取决于锁定的 ROS 2、`rclrs`、消息生成器与工作空间版本，必须按项目 CI 验证。

这一节代码密度高。目标是：读完你能自己把一个 Rust 节点接进真实的 ROS 2 系统。

## ROS 2 的四个核心概念

ROS 2（Robot Operating System 2）名字里有 "OS"，但它**不是操作系统**——它是跑在 Linux（或 RTOS）之上的一套通信框架 + 工具 + 约定。把上一节的"计算图"落地，就是 ROS 2 的四个概念：

### 节点（node）

一个节点是计算图里的一个顶点——通常是一个进程，负责一件事（一个感知节点、一个控制节点）。节点有名字（`/perception`），通过话题、服务、动作和别的节点通信。一个进程里也可以塞多个节点（component），但心智模型上"一个节点干一件事"。

### 话题（topic）与消息（message）

话题是有名字的**类型化**通道，比如 `/obstacles`。类型化很重要：一个话题上跑的所有消息必须是**同一个消息类型**，比如 `smart_driver_msgs/msg/ObstacleArray`。发布者往话题发，订阅者从话题收，多对多。消息类型用一种叫 **IDL/msg** 的中立语言定义（8.4 节详谈），ROS 2 的构建系统会为 C++、Python、Rust **各生成一套对应的结构体/类**——这就是"跨语言"的由来。

### DDS 与 rmw：ROS 2 的通信底座

ROS 2 自己**不实现网络传输**，而是委托给 **DDS（Data Distribution Service）**。ROS 2 和 DDS 之间隔了一层抽象叫 **rmw（ROS middleware interface）**，于是你能换 DDS 实现（Fast-DDS、Cyclone DDS）甚至换成非 DDS 的 Zenoh（`rmw_zenoh`，下一节主角），而**应用代码不用改**。

```text
  你的 Rust 节点代码
        │  调用
        ▼
  rclrs（Rust 客户端库）
        │  调用
        ▼
  rcl（C 语言公共层）
        │
        ▼
  rmw（中间件抽象接口）
        │  可插拔
        ▼
  rmw_cyclonedds / rmw_fastrtps / rmw_zenoh …
        │
        ▼
  真正的网络（UDP 多播 / 共享内存 / …）
```

各语言客户端（`rclcpp`、`rclpy`、`rclrs`）都坐在同一个 C 层 `rcl` 之上，所以**语义完全一致、能互通**。你的 Rust 节点和别人的 C++ 节点在同一张图里对话，毫无障碍——这是 `rclrs` 能落地的关键前提。

### QoS：每条数据流的传输策略

上一节讲过 QoS 的概念，ROS 2 把它做成发布/订阅时的一组参数。最常用的两套预设：

- **Sensor Data（传感器数据）**：`BEST_EFFORT` + 只留最新几条。给点云、图像这种高频、丢一帧无所谓的流。
- **Reliable（可靠，默认）**：`RELIABLE` + 保留一定历史。给指令、状态这种每条都重要的流。

**发布者和订阅者的 QoS 必须"兼容"才能连上**——一个经典坑：发布者是 `BEST_EFFORT`、订阅者要求 `RELIABLE`，两者**不兼容，静默地收不到数据**，还不报错。调 ROS 2 时"话题在但收不到消息"，八成是 QoS 不匹配。记住这个，能省你半天。

## 为什么 ROS 1 不适合量产，ROS 2 适合

同样叫 ROS，差别是"能不能上车"级别的。逐条对比：

| 维度 | ROS 1 | ROS 2 | 为什么对量产要命 |
|---|---|---|---|
| 架构 | 中心化 `roscore` master | **去中心化**（DDS 自动发现） | master 挂了全网瘫痪——车上不能有单点故障 |
| 实时性 | 无实时保证 | 支持实时执行器、可配调度 | 控制回路要确定性 |
| QoS | 基本没有 | **完整 QoS** | 传感器/指令流需要不同传输策略 |
| 传输 | 自研 TCPROS/UDPROS | 工业级 DDS，含共享内存 | 大数据零拷贝、跨机可靠 |
| 安全 | 无（明文、无认证） | **SROS2**（加密、认证、访问控制） | 联网的车必须防篡改 |
| 多平台 | 主要 Linux | Linux/RTOS/微控制器（micro-ROS） | 车上有各种 ECU |
| 状态 | 2025 年 EOL | 活跃，工业投入 | 没人敢基于停维护的框架造量产车 |

一句话：**ROS 1 是实验室的原型工具，ROS 2 是奔着工业化去重写的**。它把中心化 master 换成 DDS 的去中心化自动发现、补上了 QoS/实时/安全这些量产必需品。所以今天没有人会用 ROS 1 造新的量产系统——学，就学 ROS 2。

## rclrs：ROS 2 的 Rust 客户端

`rclrs` 是 **ros2-rust** 项目（github.com/ros2-rust/ros2_rust）提供的 Rust 客户端库，和 `rclcpp`（C++）、`rclpy`（Python）平级，都坐在 `rcl` 之上。它让你用地道的 Rust——`Arc`、闭包、`Result`、迭代器——写 ROS 2 节点。

现状要坦白讲（2026 年时点）：

- **能用、在真实项目里被用**，pub/sub、服务、参数、定时器都支持。
- **不在 ROS 2 官方发行版（如 Humble/Jazzy）的默认二进制里**，需要你把 `ros2_rust` 拉进 workspace、用 `colcon` 连着一起构建。它靠一个叫 `rosidl_generator_rs` 的代码生成器，把 `.msg` 变成 Rust 结构体。
- **API 仍在演进**，跨版本可能有变动。所以下面的代码我给出**结构与思路完全真实、可编译**的版本，但你落地时要以你所用 `rclrs` 版本的文档为准对齐细节。

它和纯 Rust 库不太一样：**不是 `cargo add rclrs` 就完事**，而是要放进 ROS 2 的 `colcon` 工作空间，因为消息类型的生成、和 C 层 `rcl` 的链接都由 ROS 2 构建系统接管。下面我们完整走一遍。

## 第一步：定义消息接口（.msg）

我们要传全书通用的障碍物。ROS 2 消息用 `.msg` 文件定义（IDL 的一种）。新建一个消息包 `smart_driver_msgs`：

目录结构：

```text
smart_driver_msgs/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── Obstacle.msg
    └── ObstacleArray.msg
```

`msg/Obstacle.msg`——注意它和第 1.3 节的 `Obstacle` 结构一一对应：

```text
# 一个被感知到的障碍物（对应本书 Obstacle 结构）
uint32   id            # 跟踪 ID，跨帧保持不变
uint8    obstacle_class  # 0=Unknown 1=Vehicle 2=Pedestrian 3=Cyclist
float64[3] position     # 自车坐标系 x,y,z（米）
float64[3] velocity     # 速度矢量（米/秒）
float64[3] size         # 长宽高（米）
float64  heading        # 朝向角（弧度）
float32  confidence     # 置信度 0~1
```

`msg/ObstacleArray.msg`——一帧的所有障碍物，带个头（header）放时间戳：

```text
# 一帧感知输出：一堆障碍物 + 时间戳
std_msgs/Header header
Obstacle[]      obstacles
```

`std_msgs/Header` 是 ROS 2 自带的，含时间戳（`stamp`）和坐标系名（`frame_id`）——**每个传感器消息都该带 header**，这是时间同步（3.3 节）和坐标变换（3.2 节）的基础。

`package.xml`（关键部分）：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>smart_driver_msgs</name>
  <version>0.1.0</version>
  <description>本书通用的自动驾驶消息定义</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

`CMakeLists.txt`（关键部分）——告诉构建系统把这两个 `.msg` 生成为各语言的代码：

```cmake
cmake_minimum_required(VERSION 3.8)
project(smart_driver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

当 `ros2_rust` 在工作空间里时，构建会自动为这个包生成一个 Rust crate（名字就叫 `smart_driver_msgs`），里面有 `msg::Obstacle`、`msg::ObstacleArray` 结构体。下面 Rust 代码就 `use` 它。

## 第二步：发布者节点（完整代码）

新建一个 ROS 2 的 Rust 包 `perception_node`。它模拟感知模块：每 100 ms（10 Hz）发布一帧包含一个横穿行人的 `ObstacleArray`——正是第 1.3 节的场景。

`perception_node/Cargo.toml`：

```toml
[package]
name = "perception_node"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "perception_node"
path = "src/main.rs"

[dependencies]
# rclrs 与消息包由 colcon 工作空间提供，版本随你的 ros2_rust 分支
rclrs = "*"
std_msgs = "*"
builtin_interfaces = "*"
smart_driver_msgs = "*"          # 上一步生成的消息 crate
```

它还需要一个 `package.xml` 让 `colcon` 认得（这是 ROS 2 包和纯 cargo 包的区别）：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>perception_node</name>
  <version>0.1.0</version>
  <description>Rust 感知节点：发布 ObstacleArray</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>

  <depend>rclrs</depend>
  <depend>std_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>smart_driver_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

`perception_node/src/main.rs`：

```rust,ignore
use std::sync::Arc;
use std::time::Duration;

use rclrs::{Context, Node, Publisher, QoSProfile, RclrsError};

// 由 smart_driver_msgs 包生成的 Rust 消息类型
use smart_driver_msgs::msg::{Obstacle, ObstacleArray};
use std_msgs::msg::Header;

/// 感知节点：把内部逻辑和 ROS 句柄封在一起。
/// 用 Arc 是因为发布者句柄要在定时器回调（另一处所有权）里共享。
struct PerceptionNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ObstacleArray>>,
    seq: u32, // 帧计数，用来让行人随时间横穿
}

impl PerceptionNode {
    fn new(context: &Context) -> Result<Arc<Self>, RclrsError> {
        // 1) 创建节点，名字 "perception_node"
        let node = rclrs::create_node(context, "perception_node")?;

        // 2) 在话题 "/obstacles" 上创建发布者。
        //    传感器类数据用 sensor_data QoS：best-effort、只留最新几帧。
        let publisher =
            node.create_publisher::<ObstacleArray>("/obstacles", QoSProfile::sensor_data())?;

        Ok(Arc::new(Self {
            node,
            publisher,
            seq: 0,
        }))
    }

    /// 造一帧感知结果：前方 30m 有个正在横穿的行人（第 1.3 节场景）。
    fn make_frame(&self, seq: u32) -> ObstacleArray {
        // 行人以 0.3 m/s 向车道横移，随帧数累积横向位移。
        let dt = 0.1_f64; // 10 Hz
        let lateral = -1.5 + 0.3 * (seq as f64) * dt;

        let pedestrian = Obstacle {
            id: 42,
            obstacle_class: 2, // Pedestrian
            position: [30.0, lateral, 0.0],
            velocity: [0.0, 0.3, 0.0],
            size: [0.5, 0.5, 1.7],
            heading: std::f64::consts::FRAC_PI_2, // 朝 +y，横穿方向
            confidence: 0.92,
        };

        let mut header = Header::default();
        header.frame_id = "ego".to_string();
        // 真实系统里 stamp 用 node.get_clock().now()；此处从略以保持精简。

        ObstacleArray {
            header,
            obstacles: vec![pedestrian],
        }
    }

    /// 发布一帧，并打印一行日志。
    fn publish_once(&mut self) -> Result<(), RclrsError> {
        let msg = self.make_frame(self.seq);
        let ped = &msg.obstacles[0];
        println!(
            "[perception] 发布第 {} 帧：行人#{} 位于 y={:.2}m",
            self.seq, ped.id, ped.position[1]
        );
        self.publisher.publish(msg)?;
        self.seq += 1;
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    // 初始化 ROS 2 上下文（等价于 rclcpp::init）
    let context = Context::new(std::env::args())?;
    let mut perception = Arc::try_unwrap(PerceptionNode::new(&context)?)
        .map_err(|_| ())
        .expect("独占持有");

    // 简单的 10 Hz 循环。真实项目更常用 node 的定时器（create_timer），
    // 这里用显式 sleep 让控制流一目了然。
    println!("[perception] 节点已启动，10 Hz 发布 /obstacles ...");
    let period = Duration::from_millis(100);
    while context.ok() {
        perception.publish_once()?;
        std::thread::sleep(period);
    }
    Ok(())
}
```

代码里几个值得注意的地方：

- **`Arc` 无处不在**。ROS 2 的节点、发布者、订阅者句柄天然要被多处共享（回调、定时器、多线程执行器），`rclrs` 顺着 Rust 的所有权模型，用 `Arc` 表达这种共享。这不是啰嗦，是把"这个句柄被谁持有"这件在 C++ 里靠人肉记忆的事，交给编译器管。
- **QoS 选了 `sensor_data()`**。因为发的是高频感知流，best-effort 合理。**订阅端必须用兼容的 QoS**，否则收不到——下面订阅者也用 `sensor_data()`。
- **消息就是普通 Rust 结构体**。`Obstacle { id: 42, ... }`，字段名来自 `.msg` 定义。填充、`vec!`、`&msg.obstacles[0]` 都是你熟悉的 Rust，没有魔法。

## 第三步：订阅者节点（完整代码）

再建一个包 `planning_node`，订阅 `/obstacles`，对每一帧做一个极简的"风险判断"——正是规划要干的第一件事。

`planning_node/Cargo.toml` 依赖同上（把 `name` 改成 `planning_node` 即可）。`planning_node/src/main.rs`：

```rust,ignore
use std::sync::Arc;

use rclrs::{Context, Node, QoSProfile, RclrsError, Subscription};
use smart_driver_msgs::msg::ObstacleArray;

/// 规划节点：订阅 /obstacles，对每帧做碰撞风险粗判。
struct PlanningNode {
    node: Arc<Node>,
    // 订阅句柄要一直活着，否则回调不会被触发——所以存起来。
    _subscription: Arc<Subscription<ObstacleArray>>,
}

impl PlanningNode {
    fn new(context: &Context) -> Result<Arc<Self>, RclrsError> {
        let node = rclrs::create_node(context, "planning_node")?;

        // QoS 必须与发布端兼容：发布端是 sensor_data(best-effort)，这里也用它。
        let subscription = node.create_subscription::<ObstacleArray, _>(
            "/obstacles",
            QoSProfile::sensor_data(),
            // 回调：每收到一帧就被调用一次。用 move 闭包捕获所需状态。
            move |msg: ObstacleArray| {
                Self::on_obstacles(msg);
            },
        )?;

        Ok(Arc::new(Self {
            node,
            _subscription: subscription,
        }))
    }

    /// 收到一帧障碍物的处理逻辑。
    fn on_obstacles(msg: ObstacleArray) {
        let n = msg.obstacles.len();
        println!("[planning] 收到 {} 个障碍物", n);

        for ob in &msg.obstacles {
            // 极简风险模型：前方障碍物、且有指向车道的横向速度 -> 需减速让行。
            let ahead = ob.position[0]; // 纵向距离（米）
            let lateral = ob.position[1]; // 横向位置（米），0 为车道中心
            let lateral_v = ob.velocity[1]; // 横向速度（米/秒）

            // 是否正在靠近车道中心？（横向位置与横向速度反号 = 正在往中心靠）
            let approaching = lateral * lateral_v < 0.0;

            if ahead > 0.0 && ahead < 50.0 && approaching {
                // 一个粗糙的"碰撞时间"直觉：还有多久横向到达车道中心
                let ttc_lateral = if lateral_v.abs() > 1e-3 {
                    (lateral.abs() / lateral_v.abs()) as f64
                } else {
                    f64::INFINITY
                };
                println!(
                    "  ⚠ 障碍物#{} 前方 {:.1}m 正横穿，约 {:.1}s 后到车道中心 -> 决策：减速让行",
                    ob.id, ahead, ttc_lateral
                );
            } else {
                println!("  · 障碍物#{} 前方 {:.1}m，暂无风险", ob.id, ahead);
            }
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(std::env::args())?;
    let planning = PlanningNode::new(&context)?;
    println!("[planning] 节点已启动，订阅 /obstacles ...");

    // spin：把控制权交给 ROS 2 执行器，阻塞地处理回调，直到 Ctrl-C。
    rclrs::spin(planning.node.clone())
}
```

关键点：

- **订阅句柄必须存活**。`_subscription` 前缀下划线是告诉编译器"我知道没直接读它，别警告"，但**绝不能不存它**——一旦它被 drop，订阅就注销，回调再也不触发。这是 `rclrs`（乃至所有 RAII 风格库）的常见坑：把返回的句柄丢掉 = 悄悄地不工作。
- **回调是闭包**。`move |msg| { ... }`，地道 Rust。要在回调里更新状态（比如维护一个障碍物历史），就用 `Arc<Mutex<State>>` 捕获进去——Rust 会强迫你把这个共享状态的线程安全想清楚，这正是它的价值。
- **`rclrs::spin`** 把线程交给 ROS 2 执行器（executor），它负责在消息到来时调你的回调。这对应 `rclcpp::spin`。

## 第四步：构建与运行

因为涉及消息生成和 `rcl` 链接，构建走 `colcon` 而非裸 `cargo`：

```bash
# 假设 ros2_rust 已按官方说明放进工作空间的 src/ 下
cd ~/smart_driver_ws
source /opt/ros/jazzy/setup.bash          # 你的 ROS 2 发行版
colcon build                              # 会先生成 msg、再编译 Rust 节点
source install/setup.bash

# 两个终端分别跑（都要先 source install/setup.bash）
ros2 run perception_node perception_node   # 终端 A
ros2 run planning_node   planning_node      # 终端 B
```

你会看到 A 每 100 ms 发一帧、行人的 `y` 从 -1.5 逐渐涨向 0，B 每帧打印风险判断、`ttc_lateral` 越来越小。这就是第 1.3 节那个场景，第一次以**真正的多进程、消息驱动**形态跑起来——第一部分埋的伏笔到此闭环。

还能用标准工具观察这张计算图：

```bash
ros2 topic list                 # 看到 /obstacles
ros2 topic hz /obstacles        # 确认 ~10 Hz
ros2 topic echo /obstacles      # 直接打印消息内容
rqt_graph                       # 图形化看到 perception -> /obstacles -> planning
```

**注意：这里 A 是 Rust、B 也是 Rust，但你完全可以把 B 换成一个 C++ 或 Python 节点**——只要它订阅 `/obstacles`、用兼容 QoS，就照收不误。这就是 rmw/rcl 分层带来的跨语言互通。

## 替代方案：r2r

`rclrs` 不是唯一选择。**`r2r`**（github.com/sequenceplanner/r2r）是另一套社区维护的 Rust ROS 2 绑定，风格不同：

- `r2r` 更**贴近 async**，天然和 `tokio`/`futures` 结合，订阅是一个 `Stream`、服务调用是 `Future`——如果你的节点本来就是异步架构（第 2.5 节的 tokio），`r2r` 常常更顺手。
- 它对消息的处理更"运行时"一些，构建集成方式也和 `rclrs` 有别。
- 成熟度和社区上，两者各有拥趸；`rclrs` 由 ros2-rust 社区维护，`r2r` 是独立项目。

选择建议：按所需的 actions、参数、loaned message、async、目标 ROS 发行版和稳定性逐项做兼容矩阵；不要用“官方/非官方”替代工程评估。

## 工程现实：成熟度与混合部署

坦白几句你迟早会撞上的现实：

- **`rclrs` 还不是"下载即用"**。它要你搭 `colcon` 工作空间、拉 `ros2_rust`、跑代码生成。比起 `rclcpp`/`rclpy` 的开箱即用，前期配置更折腾。API 也可能随版本变。**所以现实中的做法**：绝大多数团队是 **C++/Python 为主的 ROS 2 系统**，用 Rust 写**新的、性能或可靠性关键的节点**——一个高频融合节点、一个安全监控节点——而不是推倒重写整套。
- **混合部署是常态，也是 ROS 2 的设计初衷**。同一张计算图里，感知是 C++（要 CUDA、要 PCL）、规划试点用 Rust、可视化和脚本用 Python，它们通过话题无缝协作。你作为"会 Rust 又懂智驾"的人，最可能的切入点就是**在既有 C++ 系统里插入 Rust 节点**——这和第 1.4 节"渐进式引入 Rust"的判断完全一致。
- **消息包最好用同一个**。像我们的 `smart_driver_msgs`，C++/Python/Rust 节点共享同一份 `.msg` 定义、各自生成绑定，接口就是那份 IDL——8.4 节会把"消息即契约"这件事讲透。

> **面试题**：ROS 2 里，为什么一个 Rust 节点能和 C++ 节点在同一个话题上通信，尽管它们编译成完全不同的二进制？
> **答**：因为消息类型由中立的 `.msg`（IDL）定义，各语言客户端（rclrs/rclcpp）各自生成对应的结构体，但**线上传输的字节格式（DDS 的 CDR 序列化）是统一的**；而且它们都通过同一层 rmw/DDS 通信。语言只影响"内存里长什么样"，不影响"线上传什么"——序列化格式才是真正的契约。

## 小结

- ROS 2 四概念：**节点**（图的顶点）、**话题+消息**（类型化的边）、**DDS/rmw**（可插拔的通信底座）、**QoS**（每条流的传输策略，发布/订阅必须兼容否则静默收不到）。
- **ROS 2 适合量产、ROS 1 不适合**：去中心化（无 master 单点）、有 QoS/实时/安全（SROS2）、工业级 DDS 底座，而 ROS 1 已 EOL。
- `rclrs` 是坐在 `rcl` 之上的 Rust 客户端，可与 C++/Python 节点互通；本章给出发布者（发 `/obstacles`）和订阅者（做风险判断）的结构，项目中须锁定版本并用 CI 验证。
- 关键工程点：**句柄要用 `Arc` 共享、订阅句柄必须存活、发布/订阅 QoS 要兼容**；构建走 `colcon` 因为要生成消息、链接 `rcl`。
- **`r2r`** 是 async 风格的替代绑定；现实中 Rust 多用于**在 C++/Python 为主的 ROS 2 系统里写新的关键节点**，混合部署是常态。

下一节我们钻到底层：DDS 到底怎么工作，以及为什么 Rust 原生的 **Zenoh** 被选为下一代 rmw——顺带用 `zenoh` crate 写一对不依赖整套 ROS 2 工作空间、`cargo run` 就能跑的收发程序。
