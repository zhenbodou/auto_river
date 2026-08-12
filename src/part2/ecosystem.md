# 2.6 智驾常用 crate 生态速览

前面五章你学了语言本身。但没人从零造轮子——真实项目是站在生态（ecosystem）上的。这一章是一张**地图**：智驾开发里，哪类活儿该用哪个 crate。我不会深讲每个库的 API（那是后面各部分的事），而是给你**一句话定位**加**选型判断**，让你在开一个新项目、`cargo add` 之前，脑子里有张清晰的图。

先说个前提认知（第 1.4 节讲过）：**Rust 的智驾生态比 C++ 年轻**。C++ 有 PCL、OpenCV、Eigen 几十年积累，Rust 的对应物有些很成熟（serde、tokio、nalgebra），有些还在快速演进（推理、可视化），有些暂时得包一层 C++。知道每个库的**成熟度**和它在**你系统里的位置**，比记住 API 重要得多。这一章的价值就在这里。

## 数学与线性代数

自动驾驶本质是"几何 + 概率的实时计算"，线性代数库是地基。三个主要选择，定位很不一样：

- **`nalgebra`** — 通用线性代数库，功能最全。矩阵、向量、四元数、变换、分解、求解——卡尔曼滤波（第五部分）、坐标变换（第 3.2 节）、控制（第七部分）都会用到它。**维度可以编译期固定**（`Matrix3<f64>`），编译器帮你查维度匹配，还能优化。定位：**智驾数学计算的主力**。
- **`ndarray`** — N 维数组，对标 Python 的 NumPy。适合处理"一大块规则数据"——图像张量、批量点、神经网络的输入输出。和推理库、图像处理配合多。定位：**多维数值数组处理**。
- **`glam`** — 专为图形/游戏优化的轻量线代库，主打 3D 向量/矩阵/四元数，用 SIMD 加速，API 简洁、极快。适合坐标变换、可视化、仿真里的几何运算。定位：**快而轻的 3D 几何**。

```toml
[dependencies]
nalgebra = "0.33"   # 主力：滤波、变换、控制
ndarray = "0.16"    # 张量、批量数据、和推理配合
glam = "0.29"       # 轻量 3D 几何、可视化、仿真
```

```rust,ignore
use nalgebra::{Matrix3, Vector3};

// 一个绕 z 轴旋转 yaw 的旋转矩阵，把点从自车系转到地图系（第 3.2 节详讲）
fn rotation_z(yaw: f64) -> Matrix3<f64> {
    let (s, c) = yaw.sin_cos();
    Matrix3::new(
        c, -s, 0.0,
        s,  c, 0.0,
        0.0, 0.0, 1.0,
    )
}

fn to_map_frame(p_ego: Vector3<f64>, ego_yaw: f64, ego_pos: Vector3<f64>) -> Vector3<f64> {
    rotation_z(ego_yaw) * p_ego + ego_pos
}
```

> **选型建议**：**默认上 `nalgebra`**，它覆盖面最广、社区最大、和机器人生态（如 SLAM 库）集成好。只在明确的"图形/仿真里做海量简单 3D 运算、要极致轻快"时选 `glam`。`ndarray` 不是 `nalgebra` 的竞品——它俩定位不同，前者是"NumPy 式的大数组"，后者是"带线代语义的矩阵向量"，你的项目里很可能两个都在。

## 几何与坐标变换

坐标系和刚体变换是智驾的家常便饭（第 3.2 节整章讲）。除了直接用 `nalgebra` 的矩阵/四元数，还有专门的库：

- **`nalgebra` 的 `Isometry3`/`UnitQuaternion`** — 直接表达刚体变换（旋转+平移）和单位四元数，语义清晰、防止你手搓旋转矩阵搞错。**首选**。
- 机器人领域的 **TF 树**（坐标系变换树）概念，在 ROS 2 生态里由中间件侧提供（第八部分）。

一句话：**坐标变换优先用 `nalgebra::Isometry3` 而不是裸矩阵**——它在类型层面区分了"这是个刚体变换"，比一个 `Matrix4` 语义清楚得多，也不容易搞错平移和旋转的顺序。

## 序列化：数据的进出口

模块之间传消息、存配置、读录制数据，都要序列化（serialization）。Rust 这块非常成熟。

- **`serde`** — 序列化框架的**事实标准**，本身与格式无关，通过 `#[derive(Serialize, Deserialize)]`（第 2.3 节见过）给你的类型免费获得序列化能力。它是一个抽象层，配合各种"格式后端"用。
- **`serde_json`** — JSON 后端，人类可读，配置、调试、Web 接口首选。
- **`serde_yaml`** — YAML 后端，配置文件常用（比 JSON 更适合人写）。
- **`prost`** / **protobuf** — Protocol Buffers 的 Rust 实现，**紧凑的二进制格式 + 跨语言 schema**。车载模块间通信、和 C++/Python 节点互通、录制大规模数据时用它——省带宽、省存储、有明确的版本化 schema。定位：**高性能跨语言消息（第 8.4 节详讲）**。
- **`bincode`** — Rust 原生的紧凑二进制格式，纯 Rust 系统内部通信时简单高效（但不跨语言）。

```toml
[dependencies]
serde = { version = "1", features = ["derive"] }
serde_json = "1"
serde_yaml = "0.9"
```

```rust,ignore
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
struct PerceptionConfig {
    detector_model: String,
    confidence_threshold: f32,
    max_range_m: f64,
}

fn load_config(path: &str) -> anyhow::Result<PerceptionConfig> {
    let text = std::fs::read_to_string(path)?;
    let cfg: PerceptionConfig = serde_yaml::from_str(&text)?; // YAML -> struct
    Ok(cfg)
}
```

> **选型建议**：**配置和调试用 `serde_json`/`serde_yaml`（可读），模块间高频通信和跨语言用 `prost`/protobuf（紧凑、有 schema），纯 Rust 内部图省事用 `bincode`。** 关键洞察：`serde` 是那个不变的地基，换格式只是换后端 crate，你的类型定义和 `#[derive]` 一行不用改——这是 Rust 序列化生态最舒服的地方。

## 并发与并行

第 2.5 节的库在这里归个类：

- **`tokio`** — 异步运行时的**事实标准**。IO 密集场景（网络、云通信、多路传感器驱动、V2X）的地基。定位：**async 运行时**。
- **`rayon`** — 数据并行的神器。把一个串行迭代器改成并行只需把 `.iter()` 换成 `.par_iter()`，它自动用工作窃取（work-stealing）把活分到所有核上。CPU 密集的点云处理、批量几何运算的**首选**。定位：**"傻瓜式"数据并行**。
- **`crossbeam`** — 高性能并发原语，最常用的是它的 channel（mpmc、有界，见第 2.5 节）和无锁数据结构。定位：**并发原语工具箱**。

```toml
[dependencies]
tokio = { version = "1", features = ["full"] }
rayon = "1"
crossbeam = "0.8"
```

`rayon` 的威力值得单独展示——把点云滤波并行化几乎零成本：

```rust,ignore
use rayon::prelude::*;

// 串行版
let ground: Vec<&Point> = cloud.points.iter().filter(|p| p.z < 0.1).collect();

// 并行版——只改了一个词，rayon 自动铺满所有 CPU 核
let ground: Vec<&Point> = cloud.points.par_iter().filter(|p| p.z < 0.1).collect();
```

> **选型建议**：**IO 等待选 `tokio`，CPU 并行选 `rayon`，要 channel 选 `crossbeam`**（第 2.5 节的判据）。三者常在同一系统里各管一摊，不冲突。`rayon` 对"处理十几万个点"这类天然可并行的活儿性价比极高，几乎是白捡的加速。

## 日志与追踪

生产系统没日志等于盲飞。两个层次：

- **`log`** — 轻量的日志**门面（facade）**，提供 `info!`、`warn!`、`error!`、`debug!` 宏。它只是接口，本身不输出，需要配一个后端（如 `env_logger`、`simple_logger`）。定位：**简单日志的标准接口**。
- **`tracing`** — 更强大的**结构化、带 span 的**追踪框架。它不只记"发生了什么"，还能记"在哪个上下文里、耗时多久"——对理解并发系统里一帧数据穿过多个异步任务的完整路径极其有用。定位：**异步/并发系统的可观测性基础设施**。

```toml
[dependencies]
log = "0.4"
tracing = "0.1"
tracing-subscriber = "0.3"
```

> **选型建议**：**简单的库或工具用 `log` 就够了；复杂的、异步的、多节点的生产系统用 `tracing`**——它的 span 能让你回答"这一帧感知从进来到出障碍物到底在哪一步慢了"这种在并发系统里非常难查的问题。两者可共存：`tracing` 能接管 `log` 的输出。你手边的 `inf/` SDK 用的就是 `log`（作为一个命令行推理工具，`log` 足够），这也印证了"工具用 `log`、大型并发系统用 `tracing`"的分工。

## 深度学习推理

感知的核心是跑神经网络（第 4.3 节整章讲）。注意：**训练在 Python（PyTorch）里做，Rust 负责部署推理**——这是第 1.4 节强调的分工。Rust 侧的推理库：

- **`ort`** — ONNX Runtime 的 Rust 绑定。ONNX 是跨框架的模型交换格式，`ort` 底层是微软成熟的 ONNX Runtime C++ 引擎，**支持 CUDA、TensorRT 等硬件加速后端**。定位：**生产推理的最稳妥选择**，性能和硬件支持最好，代价是依赖一个 C++ 运行时。
- **`tract`** — **纯 Rust** 的推理引擎，支持 ONNX 和 TensorFlow 模型子集。好处是纯 Rust、易交叉编译、无 C++ 依赖，适合嵌入式/边缘部署；代价是算子覆盖和 GPU 加速不如 `ort`。定位：**纯 Rust、易部署的轻量推理**。
- **`candle`** — Hugging Face 出的极简 Rust 深度学习框架，既能推理也能做些训练，对 Transformer/LLM 类模型友好，GPU 支持在快速发展。定位：**Rust 原生 ML 框架，社区活跃、上升期**。

```toml
[dependencies]
ort = "2"      # 生产首选，硬件加速最好
# 或
tract-onnx = "0.21"  # 纯 Rust，嵌入式友好
```

> **选型建议**：**要性能和 GPU/TensorRT 加速、部署环境能带 C++ 运行时 → `ort`**（车载 Orin 这类平台的主流选择）。**要纯 Rust、无 C++ 依赖、方便交叉编译到受限设备 → `tract`**。**想在 Rust 里做前沿模型、跟社区 → `candle`**。这块是 Rust 生态里演进最快的领域之一，选型前务必查一下当前各库对你的目标模型算子和硬件的支持情况——别照着一年前的博客选。

## 中间件：把节点连成系统

第 1.3 节说了，真实系统是"并发的节点图"，把这些节点跨进程/跨机器连起来的就是中间件（第八部分整部分讲）。Rust 的选择：

- **`rclrs`** — ROS 2 社区维护的 Rust 客户端库。如果团队在 ROS 2 生态里，可用它写 Rust 节点并和现有 C++/Python 节点通信。其 API 仍在快速演进，升级前要锁版本并跑兼容测试。
- **`r2r`** — 另一个 ROS 2 的 Rust 绑定，社区维护，风格更"Rust 味"、更贴近异步。定位：**ROS 2 的社区派 Rust 绑定**。
- **`zenoh`** — **本身就是 Rust 写的**下一代通信中间件，已被 ROS 2 选为新默认传输（rmw_zenoh）。天生分布式、支持零拷贝、能跨云边端。定位：**Rust 原生的现代通信中间件，前景看好（第 8.3 节）**。

> **选型建议**：**团队已在 ROS 2 上 → 评估 `rclrs` 或 `r2r` 的功能覆盖、版本稳定性和团队维护能力；做新的分布式系统 → 评估 `zenoh`**。选型必须用真实消息大小、拓扑和故障条件做基准，不能只按语言偏好决定。

## 可视化与调试

调机器人算法，"看得见"是效率的分水岭。你需要把点云、检测框、轨迹、坐标系画出来。

- **`rerun`** — 近年冒出来的明星，**Rust 写的多模态可视化工具**，专为机器人/CV/空间计算设计。几行代码就能把点云、图像、3D 框、时间序列记录下来并在时间轴上回放。对调试感知/定位极其顺手。定位：**现代机器人数据可视化的首选**。

```toml
[dependencies]
rerun = "0.19"
```

```rust,ignore
// 把一帧点云和检测到的障碍物框丢给 rerun 可视化
fn visualize(rec: &rerun::RecordingStream, cloud: &PointCloud, obstacles: &[Obstacle]) {
    let points: Vec<[f32; 3]> = cloud.points.iter().map(|p| [p.x, p.y, p.z]).collect();
    rec.log("lidar/points", &rerun::Points3D::new(points)).ok();
    // 障碍物画成 3D 框……（第四部分详讲）
}
```

> **选型建议**：**调感知/定位/规划，直接上 `rerun`**——它是 Rust 原生、体验现代、几乎没有替代品的甜点工具。传统上大家用 ROS 的 RViz，但 `rerun` 更轻、更好用、不绑定 ROS。这是"Rust 原生工具反过来吸引整个机器人社区"的又一个例子。

## 错误处理

第 2.4 节已经讲透，这里归档：

- **`thiserror`** — 给**库**定义精确、结构化的错误类型（派生宏消灭样板）。
- **`anyhow`** — 给**应用**做万能错误 + 上下文链，顶层统一收口。

> **选型建议**：记住那句话——**库用 `thiserror`，应用用 `anyhow`**。`inf/` SDK 正是这个分工的现实样本（它作为库用 `thiserror` 定义错误枚举）。

## 一张总选型表

把全章浓缩成一张能贴在工位上的表：

| 需求 | 首选 crate | 备选 / 场景 | 一句话定位 |
|------|-----------|------------|-----------|
| 线代/滤波/变换 | `nalgebra` | `glam`（轻量3D）、`ndarray`（大数组） | 智驾数学主力 |
| 刚体变换 | `nalgebra::Isometry3` | — | 类型安全的旋转+平移 |
| 配置/可读序列化 | `serde` + `serde_yaml`/`serde_json` | — | 序列化事实标准 |
| 跨语言/高性能消息 | `prost`(protobuf) | `bincode`(纯 Rust 内部) | 紧凑二进制 + schema |
| 异步 IO 运行时 | `tokio` | — | IO 密集地基 |
| 数据并行 | `rayon` | — | 傻瓜式吃满多核 |
| channel/并发原语 | `crossbeam` | std `mpsc`(简单场景) | 有界 channel、背压 |
| 日志 | `tracing` | `log`(简单场景) | 结构化可观测性 |
| 神经网络推理 | `ort` | `tract`(纯Rust)、`candle`(前沿) | 部署推理 |
| ROS 2 节点 | `rclrs` | `r2r`(社区派) | ROS 2 里的 Rust |
| 现代中间件 | `zenoh` | — | Rust 原生、前景好 |
| 可视化调试 | `rerun` | RViz(ROS 生态) | 机器人数据可视化 |
| 库错误 | `thiserror` | — | 精确结构化错误 |
| 应用错误 | `anyhow` | — | 万能错误 + 上下文链 |

## 怎么判断一个 crate 能不能用

生态是活的，表会过时，比记住这张表更重要的是**自己评估一个 crate**的能力。上生产前，快速过一遍这几条：

- **维护活跃度**：最近有没有提交？issue 有人回吗？看 GitHub 和 [crates.io](https://crates.io) 的下载趋势。
- **版本成熟度**：还在 `0.x` 说明 API 可能变（semver 下 0.x 的小版本升级都可能破坏兼容），`1.0+` 相对稳定。
- **依赖体量**：`cargo tree` 看看它拖进来多少传递依赖——车载环境你要为每个依赖的安全和体积负责。
- **license（许可证）**：车载是商业产品，务必确认 license 兼容（MIT/Apache-2.0 最省心，GPL 要小心）。用 `cargo deny` 在 CI 里自动审查。
- **有没有 C++ 依赖**：像 `ort` 要带 C++ 运行时，会让交叉编译（第 2.1 节）复杂化——嵌入式部署尤其要掂量。

> **中高级视角**：`docs.rs` 是你的好朋友——每个发布到 crates.io 的 crate 都自动生成并托管文档。Rust 社区的文档文化很好，`cargo doc` 生成的 API 文档质量普遍高于 C++ 生态。养成"选型时先读 docs.rs、看它的示例和 `Cargo.toml` 里的 feature 开关"的习惯，比读二手博客靠谱得多。

## 小结

- 这一章是张地图，把第 2.1~2.5 章的语言机制和真实 crate 对上号，重点是**每个库在系统里的位置**和**选型判断**，不是 API 细节。
- **数学** `nalgebra`（主力）/ `glam`（轻量3D）/ `ndarray`（大数组）；**序列化** `serde` + 后端（可读用 yaml/json、跨语言用 protobuf）；**并发** `tokio`(IO) / `rayon`(CPU) / `crossbeam`(channel)；**日志** `tracing`/`log`。
- **推理** `ort`（生产首选）/ `tract`（纯Rust嵌入式）/ `candle`（前沿）；**中间件** `rclrs`/`r2r`(ROS 2) / `zenoh`（Rust 原生新贵）；**可视化** `rerun`；**错误** `thiserror`(库)/`anyhow`(应用)。
- Rust 智驾生态**有的成熟（serde/tokio/nalgebra）、有的在快速演进（推理/可视化）、有的还得包 C++**——认清成熟度再选型。
- 学会**自己评估 crate**：看维护活跃度、版本成熟度、依赖体量、license、有无 C++ 依赖；善用 `docs.rs`、`cargo tree`、`cargo deny`。

**第二部分到此结束。** 你已经有了写智驾 Rust 代码的完整语言底子——所有权让你安全高效地管内存、类型系统让你优雅建模、错误处理让你面对车载现实、并发让你搭出节点图、生态让你不重复造轮子。从下一部分开始，我们把镜头转向**智驾本身的技术内容**：第三部分先讲清楚传感器、坐标系和数据——一切感知与定位的原材料从哪来、长什么样、怎么用 Rust 表达。真正的自动驾驶，现在开始。
