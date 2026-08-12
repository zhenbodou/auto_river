# 2.3 类型系统、trait 与泛型

上一章你学会了怎么安全地管理内存。这一章讲怎么**优雅地建模问题**。Rust 的类型系统是它除所有权之外的第二大杀手锏——它让你能把"一个障碍物要么是车、要么是人、要么未知""一条传感器消息要么是图像帧、要么是点云帧"这类事实，**直接编码进类型里**，让编译器帮你保证逻辑完备、不漏分支。

如果你从 C 或 Python 来，会经历一个观念转变：在那些语言里类型系统常常是碍事的（"我就想塞个值进去，别拦我"）；而在 Rust 里，类型系统是你**主动使唤的工具**——你用它把非法状态变得根本无法表达（make illegal states unrepresentable）。对一个安全攸关的系统，这是巨大的价值。

## struct：把相关数据捆在一起

你已经见过 `struct` 了（`Obstacle`、`Pose`）。它就是把几个字段捆成一个类型。这里只补两个你该知道的变体。

**元组结构体（tuple struct）**——字段没名字，适合"新类型"包装：

```rust
/// 用新类型区分“米”和“弧度”，防止把角度当距离传错
struct Meters(f64);
struct Radians(f64);
```

这招叫 **newtype 模式**。看着啰嗦，但它让编译器帮你抓住"把方向盘转角（弧度）误传给了纵向距离（米）"这种低级但致命的单位混淆——在坐标变换和控制里，单位搞错是真实事故的来源。

**给 struct 加方法**用 `impl`（第 2.1 节已经用过）：

```rust,ignore
impl Obstacle {
    /// 关联函数（无 self），当构造器用
    pub fn new(id: u32, class: ObstacleClass) -> Self {
        Self { id, class, position: [0.0; 3], velocity: [0.0; 3],
               size: [0.0; 3], heading: 0.0, confidence: 0.0 }
    }
    /// 方法（有 &self）
    pub fn speed(&self) -> f64 {
        let [vx, vy, vz] = self.velocity;
        (vx * vx + vy * vy + vz * vz).sqrt()
    }
}
```

## enum：Rust 最被低估的特性

如果你只从 C 认识 `enum`（一堆整数常量），Rust 的 `enum` 会让你重新认识它。Rust 的 `enum` 是**代数数据类型（algebraic data type）**——每个变体可以携带**不同类型、不同数量**的数据。这是它比 C++ 表达力强得多的地方。

先看简单的（第 1.3 节定义过的障碍物类别）：

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObstacleClass {
    Vehicle,
    Pedestrian,
    Cyclist,
    Unknown,
}
```

这和 C 的枚举差不多。真正的威力在于**变体能带数据**。想象你的系统从各种传感器收消息，它们结构完全不同——图像是一大块像素、点云是一堆点、雷达是几十个目标、IMU 是几个浮点数。用一个 enum 统一表达：

```rust,ignore
/// 传感器消息：一个类型，涵盖所有传感器输出
pub enum SensorMessage {
    Camera { timestamp: f64, width: u32, height: u32, data: Vec<u8> },
    Lidar(PointCloud),
    Radar { timestamp: f64, targets: Vec<RadarTarget> },
    Imu { timestamp: f64, accel: [f64; 3], gyro: [f64; 3] },
    GnssLost,  // 有些变体根本不带数据
}
```

一个 `SensorMessage` **在任意时刻恰好是这五种之一**，编译器清清楚楚。这比 C++ 里"一个基类指针 + 一堆子类 + `dynamic_cast`"或者"一个 union + 一个手动维护的 tag"干净太多，也安全太多——那些方案里你可能忘了处理某个 tag，Rust 不会让你忘。

## 模式匹配：`match` 的完备性检查

拿到一个 `SensorMessage`，你用 `match` 拆开它。`match` 不只是 `switch` 的升级版——它有一个杀手级特性：**编译器强制你处理所有变体**。

```rust,ignore
fn dispatch(msg: &SensorMessage) {
    match msg {
        SensorMessage::Camera { width, height, .. } => {
            log::info!("图像帧 {}x{}", width, height);
        }
        SensorMessage::Lidar(cloud) => {
            log::info!("点云帧 {} 个点", cloud.points.len());
        }
        SensorMessage::Radar { targets, .. } => {
            log::info!("雷达 {} 个目标", targets.len());
        }
        SensorMessage::Imu { accel, .. } => {
            log::info!("IMU 加速度 {:?}", accel);
        }
        SensorMessage::GnssLost => {
            log::warn!("GNSS 信号丢失！");
        }
    }
}
```

假如有一天团队给 `SensorMessage` 加了个新变体 `Ultrasonic { .. }`，而你忘了在这个 `match` 里处理它——**编译器会报错**：`non-exhaustive patterns: SensorMessage::Ultrasonic not covered`。

停下来体会这件事的分量：在一个几十个模块的大系统里，加一个新传感器类型，编译器会**精确地把所有需要更新的 `match` 都给你标出来**。C++ 的 `switch` 漏了个 case 顶多给个可关闭的 warning，运行时静默走 default 甚至什么都不做——在车上，"静默漏处理一种传感器状态"可能就是一次故障。**Rust 的完备性检查把"改动引发的遗漏"从运行时隐患变成编译期清单。** 这是我最推崇的 Rust 特性之一。

模式匹配还能做很多事——解构、绑定、加守卫条件：

```rust,ignore
fn assess(obs: &Obstacle) -> &'static str {
    match obs.class {
        ObstacleClass::Pedestrian | ObstacleClass::Cyclist if obs.speed() > 1.0 =>
            "移动的弱势道路使用者，高度警惕",
        ObstacleClass::Pedestrian | ObstacleClass::Cyclist =>
            "静止的弱势道路使用者，警惕",
        ObstacleClass::Vehicle if obs.position[0] < 20.0 =>
            "近处车辆，注意",
        _ => "其它，常规处理",
    }
}
```

`|` 是"或多个模式"，`if` 是**守卫（guard）**，`_` 是"其余全部"。这套东西让复杂的分类逻辑写得又紧凑又不漏。

> **面试题**：`Option<T>` 和 `Result<T, E>` 的本质是什么？
> **答**：它们就是标准库里的普通 `enum`，没有任何魔法。`Option<T>` 是 `Some(T) | None`，`Result<T, E>` 是 `Ok(T) | Err(E)`。Rust 用"带数据的枚举 + 强制完备匹配"取代了其它语言的 null 和异常——你**必须**显式处理"没有值"和"出错了"的分支，编译器不让你假装它们不存在。这是第 2.4 节错误处理的全部基础。

## trait：Rust 的接口

`struct`/`enum` 描述"数据是什么"，**trait（特征）** 描述"能干什么"——它是 Rust 版的接口（interface），定义一组行为，让不同类型去实现。

在智驾里，"传感器"是个天然的抽象：不管是相机、激光雷达还是雷达，它们都能"读一帧数据"。定义一个 `Sensor` trait：

```rust,ignore
/// 任何传感器都能被轮询出一帧消息
pub trait Sensor {
    /// 传感器名字，用于日志
    fn name(&self) -> &str;

    /// 读一帧；读不到（还没数据/掉线）返回 None
    fn poll(&mut self) -> Option<SensorMessage>;

    /// 标称频率（Hz），有个默认实现——不是每个传感器都要重写
    fn nominal_rate_hz(&self) -> f64 {
        10.0
    }
}
```

注意 `nominal_rate_hz` 有个**默认实现**——实现这个 trait 的类型可以直接用，也可以重写。这让 trait 既能定契约又能提供公共逻辑。

给具体传感器实现它：

```rust,ignore
pub struct Lidar {
    driver_handle: u32,
    // ...
}

impl Sensor for Lidar {
    fn name(&self) -> &str { "lidar_top" }

    fn poll(&mut self) -> Option<SensorMessage> {
        // 真实实现会去读驱动；这里示意
        let cloud = read_from_driver(self.driver_handle)?; // ? 见 2.4 节
        Some(SensorMessage::Lidar(cloud))
    }

    fn nominal_rate_hz(&self) -> f64 { 10.0 } // 激光雷达通常 10 Hz
}
```

再定义一个更贴近感知的 `Detector` trait——"能从一帧数据里检出障碍物":

```rust,ignore
pub trait Detector {
    /// 从点云检出障碍物
    fn detect(&self, cloud: &PointCloud) -> Vec<Obstacle>;
}
```

你可以有多个实现：一个基于传统聚类的 `EuclideanClusterDetector`、一个基于深度学习的 `PointPillarsDetector`（第四部分详讲）。它们输入输出契约相同，可以互换。

## 派生宏：让编译器帮你写实现

你一直在用的 `#[derive(...)]` 就是让编译器**自动生成 trait 实现**的宏。手写这些实现又臭又长，`derive` 一行搞定。常用的几个：

| 派生 | 作用 | 智驾里的典型用途 |
|------|------|------------------|
| `Debug` | 用 `{:?}` 打印 | 日志、调试，几乎人人加 |
| `Clone` | 提供 `.clone()` 深拷贝 | 需要独立副本时 |
| `Copy` | 赋值时按位复制（见 2.2） | 小值类型如 `Point`、`Pose` |
| `PartialEq`/`Eq` | 支持 `==` 比较 | 比较障碍物类别、ID |
| `PartialOrd`/`Ord` | 支持 `<`、排序 | 按距离/时间排序 |
| `Hash` | 能做 HashMap 的键 | 用障碍物 ID 建索引 |
| `Default` | 提供 `T::default()` 零值 | 初始化配置、状态 |
| `Serialize`/`Deserialize` | serde 序列化（见 2.6） | 存/读配置、消息 |

```rust
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct TrajectoryPoint { /* 来自 1.3 节 */
    pub t: f64, pub x: f64, pub y: f64, pub v: f64, pub heading: f64,
}
```

> **中高级视角**：`Serialize`/`Deserialize` 来自第三方 crate `serde`，能被 `derive` 是因为 serde 提供了派生宏。这引出 Rust 生态的一个哲学：**语言核心保持精简，强大能力通过 trait + 派生宏由库提供**。你之后会给消息类型加 `#[derive(Serialize, Deserialize)]`，然后它就能免费获得 JSON、二进制、protobuf 等各种格式的序列化能力（第 8.4 节）。

## 泛型 + trait bound：一份代码，多种类型

现在把 trait 和泛型（generics）结合起来。假设你要写一个"处理任意检测器输出的后处理函数"，你不想为每种检测器各写一遍。泛型让你写一份：

```rust,ignore
/// 对任意实现了 Detector 的检测器，跑检测并按距离排序
fn detect_and_sort<D: Detector>(detector: &D, cloud: &PointCloud) -> Vec<Obstacle> {
    let mut obstacles = detector.detect(cloud);
    obstacles.sort_by(|a, b| {
        let da = a.position[0].hypot(a.position[1]);
        let db = b.position[0].hypot(b.position[1]);
        da.partial_cmp(&db).unwrap()
    });
    obstacles
}
```

`<D: Detector>` 读作"对任意类型 `D`，只要它实现了 `Detector`"。这个 `: Detector` 就是 **trait bound（trait 约束）**——它限定 `D` 必须满足什么能力，编译器才允许你在函数体里调 `detector.detect(...)`。

关键点：**泛型是编译期的**。编译器会为你实际用到的每一种 `D`（比如 `EuclideanClusterDetector`、`PointPillarsDetector`）**各生成一份专门的机器码**，这叫**单态化（monomorphization）**。结果就是：泛型代码运行起来和你手写专门版本**一样快，没有任何运行时开销**——这就是 Rust 反复标榜的"零成本抽象"。代价是编译时间变长、二进制变大（每个类型一份代码）。

trait bound 可以叠加，语法上 `where` 子句更清爽：

```rust,ignore
fn summarize<D>(detector: &D, cloud: &PointCloud) -> String
where
    D: Detector,
{
    format!("{} 检出 {} 个障碍物", "detector", detector.detect(cloud).len())
}
```

## trait object（dyn）：动态分发登场

泛型很棒，但有个硬限制：**编译期就得确定类型**。可现实里你常常做不到。比如你的传感器管线要管理一组传感器，运行时才根据配置文件决定装了哪些——可能 3 个相机 + 1 个激光雷达 + 5 个雷达。它们类型各异，你想把它们放进**同一个 `Vec`** 里统一轮询。

泛型的 `Vec<T>` 要求所有元素是**同一个** `T`，做不到"混装"。这时需要 **trait object（trait 对象）**，写作 `dyn Trait`：

```rust,ignore
/// 管理一组异构传感器——运行时才知道有哪些
pub struct SensorHub {
    sensors: Vec<Box<dyn Sensor>>, // 混装：每个元素是“某个实现了 Sensor 的东西”
}

impl SensorHub {
    pub fn poll_all(&mut self) -> Vec<SensorMessage> {
        let mut msgs = Vec::new();
        for sensor in self.sensors.iter_mut() {
            if let Some(msg) = sensor.poll() {  // 动态分发：运行时才决定调谁的 poll
                log::debug!("从 {} 收到一帧", sensor.name());
                msgs.push(msg);
            }
        }
        msgs
    }
}

fn build_hub(config: &Config) -> SensorHub {
    let mut sensors: Vec<Box<dyn Sensor>> = Vec::new();
    if config.has_lidar {
        sensors.push(Box::new(Lidar { driver_handle: 0 }));
    }
    for cam_id in &config.cameras {
        sensors.push(Box::new(Camera::new(*cam_id)));
    }
    SensorHub { sensors }
}
```

`Box<dyn Sensor>` 是一个"指向某个实现了 `Sensor` 的对象"的胖指针——它带两个指针：一个指数据、一��指该类型的 **vtable（虚函数表）**。调 `sensor.poll()` 时，程序**在运行时**通过 vtable 查到该调哪个具体实现，这叫**动态分发（dynamic dispatch）**。C++ 工程师会觉得眼熟——这正是虚函数的机制。

## dyn 还是泛型？中高级必答的取舍

这是本章最该记牢的工程判断。两种多态各有取舍：

| | 泛型 `<T: Trait>` | trait object `dyn Trait` |
|---|---|---|
| 分发时机 | 编译期（静态分发） | 运行时（动态分发，查 vtable） |
| 性能 | 零开销，可内联 | 每次调用一次间接跳转，无法内联 |
| 二进制大小 | 每个类型一份代码，膨胀 | 一份代码，紧凑 |
| 能否混装异构类型 | ❌ 不能 | ✅ 能（`Vec<Box<dyn T>>`） |
| 编译期是否需知道类型 | 是 | 否，运行时决定 |

**决策原则**：

- **类型在编译期已知、且在性能热路径上** → 用泛型。比如你的点云滤波算法，模板参数是"哪种滤波核"，编译期定死，就该用泛型让它内联到极致。
- **类型运行时才定、或需要把异构类型装一个容器** → 用 `dyn`。比如上面的 `SensorHub`、插件系统、根据配置加载的模块。

**那点动态分发的开销要紧吗？** 看频率。传感器管线每 10~100 Hz 轮询一次，每次一个间接跳转（几个纳秒）完全无所谓——这里 `dyn` 带来的灵活性远比那点开销值钱。但如果是每帧点云对 12 万个点**逐点**调用的操作，放在内层循环里，那动态分发的开销（无法内联、每点一次间接跳转）就会被放大 12 万倍，这时必须用泛型。**判据不是"动态分发慢"，而是"它在不在被高频重复执行的热路径内层"。**

> **真实项目里**：一个常见的漂亮架构是"外层 dyn、内层泛型"。传感器管线的顶层用 `Vec<Box<dyn Sensor>>` 保持配置灵活（每秒几十次调用，开销无所谓）；而每个传感器内部真正处理海量数据的算法用泛型写死，编译期充分优化。这样你既拿到了运行时的灵活性，又没在热路径上付动态分发的代价。能自然地在同一个系统里分层用好这两种多态，是中高级工程师和初学者的一个分水岭。

> **中高级视角**：不是所有 trait 都能做 trait object。一个 trait 要"对象安全（object-safe）"才能写 `dyn Trait`——粗略地说，它的方法不能有泛型参数、不能返回 `Self`（因为运行时不知道 `Self` 到底多大）。当你遇到 "the trait cannot be made into an object" 这个报错，就是撞上了这条规则。解决办法通常是把那个"不安全"的方法拆出去，或者改用泛型。知道这条，能帮你在设计 trait 时少走弯路。

## 小结

- **`struct`** 捆数据，**`enum`** 是能带数据的代数类型——用它把"传感器消息""障碍物类别"这类"多选一"的事实编码进类型，让非法状态无法表达。
- **`match`** 的完备性检查是安全利器：加个新变体，编译器会把所有漏处理的地方精确标给你，把运行时隐患变成编译期清单。
- **`trait`** 是 Rust 的接口，定义行为（`Sensor`、`Detector`），可带默认实现；**派生宏** `#[derive(...)]` 让编译器替你生成常用实现。
- **泛型 + trait bound** 是编译期多态，靠单态化做到零成本，但只能处理编译期已知的单一类型。
- **trait object `dyn Trait`** 是运行时多态，能把异构类型装进一个容器（`Vec<Box<dyn Sensor>>`），代价是一次间接跳转、无法内联。
- **选型判据**：类型编译期已知且在热路径内层 → 泛型；运行时才定或需混装 → dyn。经典架构是"外层 dyn 保灵活、内层泛型压性能"。

下一章讲**错误处理**：`Option`/`Result` 的正确用法、`?` 运算符、用 `thiserror` 和 `anyhow` 分别武装库和应用，以及一个车载系统的灵魂拷问——什么时候绝不能 `panic`，什么时候 fail-fast 反而是对的。
