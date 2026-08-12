# 3.4 用 Rust 表达传感器数据

前三章我们认识了传感器、学会了坐标变换、搞定了时间同步。这一章是第三部分的收官，也是承前启后的一环：**把所有这些概念，落成一套干净、高效、可复用的 Rust 数据结构**。

这正是 Rust 最能发光的地方。C++ 里传感器数据结构常常是一堆裸指针、手动管理的 buffer 和"谁负责释放"的口头约定；Python 里则是一堆 numpy 数组配着松散的字典。Rust 让我们既能表达出**清晰的所有权**（这块点云归谁、谁能改它），又能榨出**接近 C 的性能**（零拷贝、控制内存布局），还能靠类型系统在编译期挡掉一整类 bug。

这章解决：**如何设计点云/图像/IMU/GNSS 的数据结构、如何用统一的消息头包装它们、如何在多个模块间零拷贝地共享大数据、以及 SoA 与 AoS 的内存布局为什么能让点云处理快好几倍**。最后讲序列化——录制和回放，数据闭环的地基。全部配可运行代码。

## 第一块砖：统一的消息头

回想 3.3 的 `Stamped<T>`，它只有时间戳。但实际工程中，一条消息还必须知道自己**在哪个坐标系里**——不然 3.2 的坐标变换就无从谈起。ROS 的 `std_msgs/Header` 就是干这个的，我们照着做一个：

```rust
/// 消息头：每条传感器消息都该带的元信息。
/// 对标 ROS 的 std_msgs/Header。
#[derive(Debug, Clone)]
pub struct Header {
    /// 采集时刻的时间戳，单调时钟纳秒（见 3.3：用于排序/对齐）。
    pub stamp_ns: u64,
    /// 数据所在的坐标系名，如 "lidar_top" / "camera_front" / "ego"。
    /// 坐标变换（3.2）靠它来查"该用哪个变换"。
    pub frame_id: FrameId,
    /// 序列号，用于检测丢帧。
    pub seq: u32,
}

/// 用枚举而非裸字符串表示坐标系，编译期防拼写错误。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameId {
    LidarTop,
    CameraFront,
    RadarFront,
    Imu,
    Ego,
    Map,
}

/// 泛型消息：头 + 负载。这是 3.3 里 Stamped<T> 的完整版。
#[derive(Debug, Clone)]
pub struct Message<T> {
    pub header: Header,
    pub data: T,
}
```

两个设计选择值得说：

1. **`frame_id` 用枚举而非 `String`**。ROS 里 `frame_id` 是字符串，灵活但一个拼写错误（`"lidar_top"` vs `"lidar-top"`）就让坐标变换默默失败。我们用枚举，让编译器替你挡住这类错误。代价是新增坐标系要改枚举——对一辆车固定的传感器配置，这个代价可以接受。（真要动态配置，再退回字符串 + 校验。）
2. **`Message<T>` 泛型化**。头是通用的，负载各不相同。这样 `Message<PointCloud>`、`Message<ImageFrame>`、`Message<ImuSample>` 共享同一套头逻辑，DRY。

## IMU 与 GNSS：小而规整的数据

从简单的开始。IMU 和 GNSS 每条消息就是几个标量，直接用定长数组或 `nalgebra` 向量，`Copy` 得起：

```rust,ignore
use nalgebra::Vector3;

/// 一条 IMU 采样。数据小，直接 Copy。
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub angular_velocity: Vector3<f64>,     // 陀螺仪 rad/s
    pub linear_acceleration: Vector3<f64>,  // 加速度计 m/s^2（含重力）
    /// 可选：厂商已解算好的姿态（单位四元数）。没有就 None。
    pub orientation: Option<nalgebra::UnitQuaternion<f64>>,
}

/// GNSS 定位。注意 fix_type——3.1 强调过必须检查它。
#[derive(Debug, Clone, Copy)]
pub struct GnssFix {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub fix_type: FixType,
    pub hdop: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixType {
    NoFix,
    Single,    // 米级
    Dgps,
    RtkFloat,  // 分米级，未收敛
    RtkFixed,  // 厘米级，可信
}

impl GnssFix {
    /// 只有 RTK 固定解才配用于厘米级定位。类型系统 + 显式检查双保险。
    pub fn is_precise(&self) -> bool {
        self.fix_type == FixType::RtkFixed
    }
}
```

`ImuSample` 是 `Copy` 的，因为它小（几十字节），复制比借用还省心。这跟接下来的点云形成鲜明对比——**数据大小决定所有权策略**，这是 Rust 建模的核心直觉。

## 图像：大 buffer 与像素格式

图像是第一个"大家伙"，一帧几 MB。它不能 `Copy`（复制几 MB 太贵），像素数据放在堆上的 `Vec<u8>`：

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    Rgb8,   // 每像素 3 字节
    Bgr8,
    Gray8,  // 每像素 1 字节
    Yuyv,   // 每 2 像素 4 字节
}

impl PixelFormat {
    /// 每像素字节数（YUYV 用平均值表达，实际按 2 像素一组）。
    pub fn bytes_per_pixel(&self) -> usize {
        match self {
            PixelFormat::Rgb8 | PixelFormat::Bgr8 => 3,
            PixelFormat::Gray8 => 1,
            PixelFormat::Yuyv => 2,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ImageFrame {
    pub width: u32,
    pub height: u32,
    pub format: PixelFormat,
    /// 行优先像素缓冲。行与行之间可能有对齐填充，故显式存 step。
    pub data: Vec<u8>,
    pub step: usize, // 每行字节数（>= width * bytes_per_pixel）
}

impl ImageFrame {
    /// 借用某一行的像素，零拷贝。返回切片，不复制数据。
    pub fn row(&self, y: u32) -> &[u8] {
        let start = y as usize * self.step;
        let len = self.width as usize * self.format.bytes_per_pixel();
        &self.data[start..start + len]
    }
}
```

那个 `step`（也叫 stride/pitch）是实战细节：很多相机驱动和 GPU 要求每行按 4/16/64 字节对齐，于是行尾有填充，`step > width * bpp`。硬编码 `width * bpp` 去索引，遇到带填充的图像就会错位——一个经典的"图像斜了"bug。存显式 `step` 是专业做法。

## 点云：这一章的性能主角

点云是数据量和性能压力最大的传感器数据，也是 Rust 内存布局知识最能发挥的地方。我们先给最直观的表示，再讲为什么它在高性能场景下不够用。

### AoS：直觉的表示

```rust
#[derive(Debug, Clone, Copy)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

/// 数组的结构体... 不，是"结构体的数组"（Array of Structs）。
pub struct PointCloudAoS {
    pub points: Vec<PointXYZI>,
}
```

这叫 **AoS（Array of Structs，结构体数组）**：内存里是 `x0 y0 z0 i0 | x1 y1 z1 i1 | ...`，一个点的四个字段紧挨着。直觉、好写、访问单个点的所有字段很自然。绝大多数教程到这就停了。但性能工程师会追问一句……

### SoA：为性能而生的布局

考虑一个极常见的操作：**把整帧点云做坐标变换（3.2 的 $Rp+t$），只用到 x/y/z，不碰 intensity**。或者：**统计所有点的平均强度，只用 intensity，不碰 xyz**。

在 AoS 里，你想连续读所有的 `x`，但内存里 `x` 之间隔着 `y z intensity`。CPU 从内存搬数据是按**缓存行（cache line，通常 64 字节）**整块搬的。读一个 `x`（4 字节），却顺带把它后面的 `y z intensity`（12 字节）也拖进缓存——如果这次操作用不上它们，这 12 字节的带宽就浪费了。而且 SIMD（一条指令处理 8 个 `f32`）需要数据连续排列，AoS 里 `x` 不连续，SIMD 用不上。

**SoA（Structure of Arrays，数组结构体）**反过来：每个字段各存一个连续数组。

```rust
/// SoA：每个分量一个连续数组。
pub struct PointCloudSoA {
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    pub z: Vec<f32>,
    pub intensity: Vec<f32>,
    // 不变式：四个 Vec 长度相等。
}

impl PointCloudSoA {
    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// 平移变换只碰 x/y/z：三个数组连续，缓存友好、可 SIMD、可自动向量化。
    pub fn translate(&mut self, dx: f32, dy: f32, dz: f32) {
        for v in &mut self.x { *v += dx; }
        for v in &mut self.y { *v += dy; }
        for v in &mut self.z { *v += dz; }
        // intensity 一个字节都没被读进缓存——零浪费。
    }

    /// 平均强度：只顺序扫 intensity 数组，其它内存完全不碰。
    pub fn mean_intensity(&self) -> f32 {
        if self.intensity.is_empty() { return 0.0; }
        self.intensity.iter().sum::<f32>() / self.intensity.len() as f32
    }
}
```

现在 `x` 全部连续排列：整帧遍历时缓存命中率高，编译器还能**自动向量化（auto-vectorization）**成 SIMD 指令。对几十万点的点云，只做几何运算的热路径上，SoA 相比 AoS 常有**数倍**的吞吐提升。这不是玄学，是缓存和内存带宽的物理。

> **面试题**：处理点云用 SoA 还是 AoS？
> **答**：看访问模式。**批量只碰部分字段**（坐标变换、滤波、统计）用 SoA——缓存友好、可 SIMD。**逐点用到全部字段**（比如按点分类、序列化单个点）AoS 更自然。高性能点云库（PCL 内部、各家自研）大量用 SoA 或 SoA/AoS 混合。没有银弹，工程上按热点操作的访问模式选，甚至两种表示并存、按需转换。

> **真实项目里**：字段选择本身也是内存优化。`PointXYZI` 是 16 字节，恰好对齐；如果你加个 `f64 timestamp` 就变 24 字节还可能引入 padding，几十万点乘起来就是几 MB 的差别，直接影响你能不能塞进 L2 缓存。工程上会精打细算每个字段的类型和顺序（`f32` 而非 `f64`——激光精度根本用不到 `f64`）。

## 零拷贝共享：Arc 让多模块共读一帧点云

现在来个真实场景。一帧点云进来后，**多个下游同时要用它**：地面分割要用、聚类要用、录制模块要存盘。它们大多**只读**。

最蠢的做法是给每个模块 `clone()` 一份——几 MB 的点云复制三四遍，内存和带宽双重浪费。Rust 的答案是 **`Arc<T>`（Atomically Reference Counted，原子引用计数）**：数据只存一份在堆上，各模块持有指向它的 `Arc`，`clone` 一个 `Arc` 只是把引用计数 +1（几纳秒），根本不碰底层几 MB 数据。所有 `Arc` 都释放后，数据自动回收。这就是**零拷贝共享（zero-copy sharing）**。

```rust,ignore
use std::sync::Arc;

/// 一帧点云消息，负载用 Arc 包起来，随便克隆分发。
pub type PointCloudMsg = Message<Arc<PointCloudSoA>>;

fn ground_segmentation(cloud: Arc<PointCloudSoA>) -> Vec<bool> {
    // 只读，clone 进来的 Arc 只是计数 +1，没有复制点云数据。
    (0..cloud.len()).map(|i| cloud.z[i] < -1.5).collect() // 粗暴的地面判据示意
}

fn cluster(cloud: Arc<PointCloudSoA>) -> usize {
    cloud.len() // 占位
}

fn dispatch(msg: &PointCloudMsg) {
    // Arc::clone 是廉价的计数操作，两个下游共享同一份底层数据。
    let g = ground_segmentation(Arc::clone(&msg.data));
    let c = cluster(Arc::clone(&msg.data));
    println!("地面点 {} 个，聚类 {}", g.iter().filter(|&&b| b).count(), c);
}
```

**如果某个下游需要修改怎么办？** `Arc` 默认是共享只读的。三条路：

- 用 `Arc::make_mut`：如果只有你一个持有者就地改，否则**写时复制（copy-on-write）**克隆一份再改——只在真要写时才付出复制代价。
- 需要多线程共享可变：`Arc<Mutex<T>>` 或 `Arc<RwLock<T>>`（读多写少用 `RwLock`）。
- 更常见的模式：下游不改原数据，而是**产出新数据**（分割结果、聚类标签），原点云始终只读共享。这是最干净的数据流，也符合"传感器数据是不可变事实"的心智模型。

> **中高级视角**：真正的极致零拷贝还包括**避免从驱动到用户态的那次复制**——用 `mmap`、DMA、或中间件的共享内存传输（第八部分的 Zenoh/DDS 支持进程间共享内存零拷贝）。`Arc` 解决的是**同进程内跨模块**的零拷贝，进程间零拷贝是中间件的活儿。分清这两层。

## 序列化：录制与回放的地基

数据结构设计好了，还得能**存下来、再读回来**——这就是序列化。它是**数据闭环**的地基：车上跑的时候把所有传感器消息录下来（rosbag、MCAP 等格式），回到办公室**回放（replay）**，就能在真实数据上反复调试、复现那个诡异的 bug、做回归测试。没有录制回放，你连"昨天那个问题"都没法重现。

Rust 生态里两条主线：

### serde：通用序列化框架

`serde` 是 Rust 序列化的事实标准，加个 derive 就能把结构体转成 JSON、YAML、bincode 等等：

```toml
[dependencies]
serde = { version = "1", features = ["derive"] }
serde_json = "1"
bincode = "1"
```

```rust,ignore
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuRecord {
    pub stamp_ns: u64,
    pub gx: f64, pub gy: f64, pub gz: f64,
    pub ax: f64, pub ay: f64, pub az: f64,
}

fn roundtrip(sample: &ImuRecord) -> ImuRecord {
    // 存盘/传输用二进制 bincode：紧凑、快。调试看内容用 JSON。
    let bytes = bincode::serialize(sample).unwrap();
    bincode::deserialize(&bytes).unwrap()
}
```

**JSON 给人看（调试、配置），bincode/二进制给机器存（录制、传输）**——JSON 把一个 `f64` 存成十几个字符，几十万点的点云用 JSON 存盘会大到离谱且慢，二进制才是正解。

### protobuf / prost：跨语言、带 schema 演进

当数据要**跨语言**（车上 Rust，云端 Python 分析）或**长期存储需要兼容旧格式**时，`serde` 的自描述格式就不够了，你需要 **protobuf**——用 `.proto` 文件显式定义 schema，字段带编号，支持向后/向前兼容的**schema 演进**（加字段不破坏旧数据）。Rust 里用 `prost`：

```proto
// sensor.proto
syntax = "proto3";
message PointCloudProto {
  uint64 stamp_ns = 1;
  string frame_id = 2;
  repeated float x = 3;   // 天然就是 SoA 布局！
  repeated float y = 4;
  repeated float z = 5;
  repeated float intensity = 6;
}
```

注意 protobuf 的 `repeated float` 编码本身就是 SoA 式的——每个字段一个连续数组，和我们前面的 `PointCloudSoA` 天然契合。`prost` 会把这个 `.proto` 生成对应的 Rust 结构体，你的录制文件就能被任何支持 protobuf 的语言读写。这正是**为什么工业界的传感器录制格式（如基于 protobuf 的 MCAP）能同时服务 Rust 采集端和 Python 分析端**。

> **本书关联**：你工作环境里那个 `inf/` 目录（把 Python 推理流水线移植成 Rust 的 SDK），跑的正是"读录制数据 → 喂模型推理 → 产出结构化结果"的离线流程。这类离线推理/评测**必须**建立在稳定的序列化格式之上——录制端和推理端对同一份 `.proto` 达成一致，才能让 Rust 采集的数据被 Rust（或 Python）推理端正确读出。数据模型的设计，直接决定了数据闭环转不转得动。

### 选型一句话

- **同进程传递**：`Arc<T>`，根本不序列化。
- **本地录制/回放**：bincode 或 MCAP，紧凑快。
- **跨语言/长期兼容**：protobuf（prost）。
- **人要读的配置/调试**：JSON/YAML。

## 把一切串起来：一帧的生命

用一段伪代码收束整个第三部分，看这些数据结构如何协同：

```rust,ignore
// 1. 驱动产出原始点云，打上头（3.3 的时间戳 + 3.2 的 frame_id）
let msg: PointCloudMsg = Message {
    header: Header { stamp_ns: now_ns(), frame_id: FrameId::LidarTop, seq: 1024 },
    data: Arc::new(raw_cloud_soa),   // 大数据用 Arc，为零拷贝分发
};

// 2. 时间对齐（3.3）：和相机、雷达凑成同一时刻的帧组
//    time_sync_buffer.push(...) 输出对齐的 msg 组

// 3. 坐标变换（3.2）：查 frame_id 对应的 lidar_to_ego，把点云搬到车体系
//    (SoA 布局让这步缓存友好、可 SIMD)

// 4. 零拷贝分发（本章）：Arc::clone 给感知各下游，各自只读

// 5. 录制（本章）：序列化落盘，供日后回放调试

// 至此，一帧原始点云变成了对齐、统一坐标、可共享、可回放的"事实"，
// 交给第四部分的感知去理解它。
```

## 小结

- 用统一的 **`Header`（时间戳 + `frame_id` + seq）+ 泛型 `Message<T>`** 包装所有传感器消息；`frame_id` 用枚举，让编译器挡住坐标系拼写错误。
- **数据大小决定所有权策略**：IMU/GNSS 小，`Copy`；图像/点云大，堆上 `Vec`，绝不随意 `clone`。图像记得存显式 `step` 应对行填充。
- **SoA vs. AoS**：批量只碰部分字段（坐标变换、滤波、统计）用 **SoA**，缓存友好、可 SIMD、常快数倍；逐点用全字段用 AoS。按热点访问模式选，是中高级性能工程的分水岭。
- **`Arc<T>` 做同进程零拷贝共享**：多个只读下游共用一份点云，`clone` 只增计数不复制数据；需可变时用 `make_mut`（写时复制）或 `Arc<RwLock<T>>`。进程间零拷贝是中间件的活儿。
- **序列化撑起数据闭环**：JSON 给人看、bincode/MCAP 本地录制、protobuf(prost) 跨语言与 schema 演进。protobuf 的 `repeated` 天然是 SoA。
- 第三部分至此闭环：一帧数据从原始字节，经**时间对齐、坐标统一、零拷贝共享、序列化录制**，变成干净可靠的"世界事实"。

第三部分到此结束。你已经掌握了传感器的物理直觉、坐标变换的数学、时间同步的工程、以及表达数据的 Rust 功底——这四样是感知、定位、规划的**共同地基**。翻开第四部分，我们要在这块地基上盖第一座大楼：**感知**，让车真正"看懂"它周围的世界。
