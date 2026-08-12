# 8.3 Zenoh、DDS 与零拷贝通信

上一节我们把 ROS 2 当"应用框架"用，但刻意没深挖它脚下的传输层。这一节钻进去：**DDS 到底是什么**、它的发现机制和 QoS、以及为什么一个 **Rust 原生**的新家伙 **Zenoh** 被 ROS 2 选为下一代默认中间件（`rmw_zenoh`）。然后我们用 `zenoh` crate 写一对**完整、`cargo run` 就能跑**的收发程序——注意，和上一节不同，这次**不依赖整套 ROS 2 工作空间**，纯 Rust。最后讲清对大点云/图像至关重要的**零拷贝共享内存**，以及 Zenoh 独有的 storage/query。

这一节和 8.2 是"应用层 vs 传输层"的关系。8.2 教你写节点，这一节教你理解节点脚下的地基，并让你能脱离 ROS 单独用 Zenoh——很多团队正是这么用它的。

## DDS：ROS 2 脚下的工业级底座

**DDS（Data Distribution Service，数据分发服务）** 是一个由 OMG 组织标准化的中间件规范，源自国防、航空、工业控制——那些"通信断了会死人"的领域。ROS 2 默认就跑在某个 DDS 实现上（Fast-DDS、Cyclone DDS 等）。它的核心特征：

### 去中心化的自动发现（discovery）

这是 DDS 相对 ROS 1 最大的进步。ROS 1 有个中心 `master` 记录"谁发什么、谁收什么"，master 挂了全网瘫痪。DDS **没有中心**：每个参与者（participant）启动时，往网络（默认用 **UDP 多播 multicast**）广播"我是谁、我发/收哪些话题"，其他参与者听到后自动建立连接。

```text
   节点A 启动 ──多播──▶ "我发 /obstacles"
   节点B 启动 ──多播──▶ "我收 /obstacles"   ──▶ A、B 自动配对，直连传数据
   （没有任何中心服务器参与）
```

好处是无单点故障、即插即用。**代价**要记住：发现是有成本的。每多一个节点、每多一个话题，参与者之间就要多交换一轮发现信息。在一个有**几百个节点、上千个话题**的大型自动驾驶系统里，发现流量会变得可观，启动时"全网互相认识"要花时间，甚至出现发现风暴。这是 DDS 在超大规模下被诟病的点，也是 Zenoh 想解决的问题之一。

### 丰富的 QoS

DDS 的 QoS 策略比上一节讲的多得多：`RELIABILITY`（可靠/尽力）、`DURABILITY`（晚来的订阅者要不要补发历史）、`HISTORY`（留几条）、`DEADLINE`（承诺的发布周期）、`LIVELINESS`（存活检测）、`LATENCY_BUDGET`、`PARTITION`（逻辑分区）……ROS 2 的 QoS 就是这些的一个子集封装。丰富意味着**强大但复杂**——配错了（尤其 reliability/durability 不兼容）就静默连不上，是 DDS 使用者的经典痛点。

### 传输：同机走共享内存，跨机走 UDP

成熟的 DDS 实现会**自动优化传输**：同一台机器上的两个参与者，可以走共享内存（避免网络栈）；跨机器才走 UDP。这正是上一节说的"透明传输选择"。但要开启共享内存往往需要额外配置和特定插件（如 Fast-DDS 的 `iceoryx` 集成），并不总是开箱即用。

一句话总结 DDS：**成熟、工业级、QoS 极丰富、有认证背书（对 ISO 26262 友好）**，但**配置复杂、发现在大规模下偏重、共享内存要费劲配**。它是今天 ROS 2 量产系统的默认与主流。

## Zenoh：Rust 原生的下一代

**Zenoh**（发音 /zeno/，Eclipse 基金会项目）是一个用 **Rust 从头写**的通信中间件。它野心比 DDS 大：不只做"局域网内节点间 pub/sub"，而是想统一 **data in motion（在传的数据，pub/sub）、data at rest（存着的数据，storage/query）、computation（计算）** 三件事，并且能**跨云、边、端**——从一颗微控制器一直连到云端数据中心，用同一套抽象。

为什么它被 ROS 2 社区看中、做成 `rmw_zenoh`（可替换 DDS 的一个 rmw 实现）？

- **轻**。DDS 的发现是"人人互相认识"，Zenoh 引入**路由器（router）**做可选的中介，发现流量可以收敛，天然适合大规模和跨网段。低带宽、高延迟的链路（比如车-云）上，Zenoh 明显更省。
- **拓扑灵活**。DDS 假设一个扁平的局域网（多播）。Zenoh 支持 peer-to-peer、client-router、mesh 各种拓扑，能自然跨越"车内局域网 + 4G/5G + 云"这种多段异构网络——这正是"车路云一体"场景要的。
- **Rust 原生**。整个协议栈是 Rust 写的，无 GC、内存安全、且 Rust API 是一等公民（不是 C 库的绑定）。对我们这本书，这意味着**你用 Zenoh 时写的是地道 Rust，没有 FFI 的别扭**。也呼应第 1.4 节："Zenoh 本身就是 Rust 写的"。
- **内建 storage/query 和共享内存**。后面详述。

代价（保持清醒）：**生态比 DDS 年轻**，功能安全认证的积累不如 DDS 深，工具链和第三方集成还在长。所以现状是：**DDS 仍是量产默认，Zenoh 是强劲的、增长很快的下一代选项**，尤其在跨网络拓扑和 Rust 团队里。

## 用 zenoh crate 写发布/订阅（完整可运行）

好消息：**Zenoh 可以完全脱离 ROS 2 单独用**，就是一个普通 Rust crate，`cargo run` 就跑。很多团队直接拿 Zenoh 当通信层，不碰 ROS。我们就来写一对——继续用障碍物场景。

新建项目 `zenoh_demo`，`Cargo.toml`：

```toml
[package]
name = "zenoh_demo"
version = "0.1.0"
edition = "2021"

[dependencies]
zenoh = "1.0"                       # Rust 原生中间件
tokio = { version = "1", features = ["full"] }   # zenoh 1.x 是 async 的
serde = { version = "1", features = ["derive"] }
serde_json = "1"                    # 先用 JSON 把消息序列化成字节（8.4 会换成更快的格式）

[[bin]]
name = "pub"
path = "src/pub.rs"

[[bin]]
name = "sub"
path = "src/sub.rs"
```

Zenoh 传的是**字节**（`ZBytes`），它不规定你用什么序列化格式——这点和 ROS 2（钦定 CDR）不同，更自由也更需要你自己约定。我们先用 JSON 把 `Obstacle` 变字节（下一节 8.4 会讲为什么生产上要换成 prost/CDR 这类二进制格式）。

先定义共享的消息类型，放 `src/obstacle.rs`（两个 bin 都 `include!` 或做成小 lib；为简明这里各自内联同一份定义思路，实际项目应放进一个 lib crate）：

```rust,ignore
use serde::{Deserialize, Serialize};

/// 与第 1.3 节 Obstacle 对应，加了 serde 派生以便序列化成字节。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obstacle {
    pub id: u32,
    pub class: u8, // 0=Unknown 1=Vehicle 2=Pedestrian 3=Cyclist
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub size: [f64; 3],
    pub heading: f64,
    pub confidence: f32,
}
```

### 发布者 `src/pub.rs`

```rust,ignore
use std::time::Duration;

use serde::{Deserialize, Serialize};

// 为让每个 bin 自包含，这里内联消息定义；真实项目请放进共享 lib。
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Obstacle {
    id: u32,
    class: u8,
    position: [f64; 3],
    velocity: [f64; 3],
    size: [f64; 3],
    heading: f64,
    confidence: f32,
}

#[tokio::main]
async fn main() {
    // 1) 打开一个 Zenoh 会话。默认配置会自动发现同网段的其他 peer。
    let session = zenoh::open(zenoh::Config::default())
        .await
        .expect("打开 zenoh 会话失败");

    // 2) 声明一个发布者，绑定到 key expression "smart_driver/obstacles"。
    //    Zenoh 的 key 是层级化的（用 / 分隔），订阅端可以用通配符匹配。
    let publisher = session
        .declare_publisher("smart_driver/obstacles")
        .await
        .expect("声明发布者失败");

    println!("[pub] 开始以 10 Hz 发布 smart_driver/obstacles ...");
    let period = Duration::from_millis(100);
    let dt = 0.1_f64;

    for seq in 0.. {
        // 复现第 1.3 节：前方 30m 一个正横穿的行人。
        let lateral = -1.5 + 0.3 * (seq as f64) * dt;
        let pedestrian = Obstacle {
            id: 42,
            class: 2,
            position: [30.0, lateral, 0.0],
            velocity: [0.0, 0.3, 0.0],
            size: [0.5, 0.5, 1.7],
            heading: std::f64::consts::FRAC_PI_2,
            confidence: 0.92,
        };

        // 3) 序列化成字节再 put。zenoh put 接受任何能变成 ZBytes 的东西，
        //    这里我们给它一段 JSON 字节。
        let bytes = serde_json::to_vec(&pedestrian).expect("序列化失败");
        publisher.put(bytes).await.expect("发布失败");

        println!("[pub] 第 {seq} 帧：行人#42 y={:.2}m", pedestrian.position[1]);
        tokio::time::sleep(period).await;
    }
}
```

### 订阅者 `src/sub.rs`

```rust,ignore
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Obstacle {
    id: u32,
    class: u8,
    position: [f64; 3],
    velocity: [f64; 3],
    size: [f64; 3],
    heading: f64,
    confidence: f32,
}

#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default())
        .await
        .expect("打开 zenoh 会话失败");

    // 声明订阅者。可以用通配符，比如 "smart_driver/**" 订阅该前缀下所有话题。
    let subscriber = session
        .declare_subscriber("smart_driver/obstacles")
        .await
        .expect("声明订阅者失败");

    println!("[sub] 订阅 smart_driver/obstacles，等待数据 ...");

    // recv_async 每收到一个 Sample 返回一次。Sample 里有 key、payload、时间戳等。
    while let Ok(sample) = subscriber.recv_async().await {
        // 取出 payload 字节，反序列化回 Obstacle。
        let bytes = sample.payload().to_bytes();
        match serde_json::from_slice::<Obstacle>(&bytes) {
            Ok(ob) => {
                let ahead = ob.position[0];
                let lateral = ob.position[1];
                let lateral_v = ob.velocity[1];
                let approaching = lateral * lateral_v < 0.0;

                if ahead > 0.0 && ahead < 50.0 && approaching {
                    let ttc = if lateral_v.abs() > 1e-3 {
                        lateral.abs() / lateral_v.abs()
                    } else {
                        f64::INFINITY
                    };
                    println!(
                        "[sub] ⚠ 障碍物#{} 前方 {:.1}m 横穿，约 {:.1}s 到车道中心 -> 减速让行",
                        ob.id, ahead, ttc
                    );
                } else {
                    println!("[sub] · 障碍物#{} 前方 {:.1}m，暂无风险", ob.id, ahead);
                }
            }
            Err(e) => eprintln!("[sub] 反序列化失败: {e}"),
        }
    }
}
```

### 跑起来

```bash
cargo run --bin sub    # 终端 A
cargo run --bin pub    # 终端 B
```

两个进程**不需要任何配置服务器**——Zenoh 默认在同网段自动发现彼此，A 立刻开始收到 B 发的障碍物。对比 8.2 的 ROS 2 版本，你会发现：**逻辑一模一样，但没有 colcon、没有 package.xml、没有消息生成步骤**，就是纯 cargo。这就是"Zenoh 作为一个普通 Rust 库"的体验。

几个和 ROS 2 心智模型的对应关系：

- ROS 2 的 **topic** ↔ Zenoh 的 **key expression**（层级化的键，`smart_driver/obstacles`），且 Zenoh 的键**支持通配符订阅**（`smart_driver/**`），比 ROS 话题更灵活。
- ROS 2 的 **消息类型系统** ↔ Zenoh **不管你传什么**，payload 就是字节，序列化格式你自己定（我们用了 JSON，生产换二进制）。自由但要自律。
- Zenoh 天然 **async**（基于 tokio），和第 2.5 节的异步生态、以及 `r2r` 那种异步 ROS 绑定很搭。

## 零拷贝共享内存：大点云/图像的生死线

现在讲这一节最"值钱"的概念。回到 8.1 的数量级：一帧点云 2 MB、一帧图像 6 MB，感知要 10~30 Hz 地在多个进程间流转。如果每次传递都要**序列化到一块新内存 → 通过 socket 拷贝 → 对端反序列化到又一块新内存**，光内存拷贝和序列化就能吃掉毫秒级时间、并制造大量内存分配抖动（破坏确定性）。

**零拷贝（zero-copy）共享内存**的思路：同一台机器上的发布者和订阅者，**共享同一块物理内存**（POSIX shared memory）。发布者把点云直接写进这块共享内存，然后**只把"这块内存在哪"这个几十字节的引用发出去**；订阅者拿到引用后，**直接读那块内存**——数据全程只存在一份，没有任何拷贝。

```text
  普通传输：                          零拷贝共享内存：
  [发布者内存: 点云] --序列化-->        [共享内存: 点云]  ← 发布者写
       [socket 拷贝]                        ▲       ▲
  [订阅者内存: 点云] <--反序列化--         写|       |读（同一块！）
   （数据被拷贝了 2~3 次）              发布者      订阅者
                                     只在网络上传一个"指针"（引用）
```

延迟和这块数据的大小**几乎无关**了——因为你传的是引用，不是数据。对 6 MB 图像，这可能是从 2 ms 降到几微秒的差别，也是"能不能 30 Hz 实时"的分水岭。

这在纯 Rust 世界里其实我们早就见过雏形：8.1 说的进程**内**用 `Arc<PointCloud>` 让多线程共享一份点云、零拷贝。共享内存就是把这个思想**跨进程**——只不过跨进程不能直接传 Rust 的 `Arc` 指针（各进程地址空间独立），要靠操作系统的共享内存机制搭桥。

Zenoh 内建了共享内存支持（DDS 也有，但 Zenoh 是 Rust 原生的）。开启方式是给 crate 加 feature：

```toml
zenoh = { version = "1.0", features = ["shared-memory"] }
```

用法的**概念**是这样（具体 API 随版本演进，落地请对照你所用版本的 `z_pub_shm` 示例）：

```rust,ignore
// 概念示意：真实 API 名称/签名以 zenoh 版本文档为准
// 1) 建一个共享内存提供者（一块预分配的内存池）
let shm_provider = /* ShmProviderBuilder：基于 POSIX 共享内存，指定池大小 */;

// 2) 从池里分配一块缓冲区，直接把点云写进去（不额外拷贝）
let mut shm_buf = shm_provider.alloc(cloud.byte_size()).await?;
shm_buf.copy_from_slice(cloud.as_bytes());   // 唯一一次写入

// 3) put 这个共享内存缓冲区。同机订阅者收到的是对同一块内存的引用，
//    Zenoh 自动判断：同机 -> 走共享内存零拷贝；跨机 -> 回退到网络传输。
publisher.put(shm_buf).await?;
```

三个工程要点：

1. **它只在同机有效**。跨机器物理内存不可能共享，Zenoh 会自动回退到普通网络传输。所以零拷贝优化的是**同一域控制器内多进程**这个最常见、也最吃带宽的场景。
2. **生命周期要管好**。共享内存里的点云，得等所有读它的订阅者都读完了才能回收——这正是引用计数要解决的问题，Zenoh 的 SHM buffer 内部处理了这个。但你要意识到：**发布者不能立刻覆盖刚发出去的缓冲区**，否则订阅者读到一半数据被改了（撕裂）。池要够大以容纳"在途"的几帧。
3. **它和序列化格式的关系**。零拷贝最爱的是**本身就是"内存布局即传输布局"的格式**（FlatBuffers、Cap'n Proto，见 8.4）——因为不需要"反序列化"这一步，订阅者拿到内存指针就能直接当结构体读。如果你用 JSON，即使内存共享了，对端还得 parse，零拷贝的价值就打了折。所以**零拷贝共享内存 + 零拷贝序列化格式**才是绝配，8.4 会把这条线接上。

> **面试题**：零拷贝共享内存能让"发一帧 6MB 图像"的延迟几乎和图像大小无关，为什么？它有什么前提和风险？
> **答**：因为传输的是对共享内存块的引用（几十字节），而非数据本身，延迟由引用大小决定而非数据量。前提是收发双方在**同一台机器**（跨机无法共享物理内存，会回退到网络传输），且最好配合零拷贝序列化格式（否则对端仍要反序列化）。风险是**内存生命周期与并发写**：必须保证订阅者读取期间发布者不覆盖该缓冲区（数据撕裂），因此需要引用计数管理和足够大的内存池容纳在途帧。

## Zenoh 独有的一招：storage 与 query

DDS 和 ROS 2 本质上只做 **data in motion**——数据流过去就没了，晚加入的订阅者收不到之前的（除非用 durability QoS 勉强补一点历史）。Zenoh 把 **data at rest** 也纳进同一套抽象：

- **Storage（存储）**：你可以让某些 key 的数据被**持久化**（存内存、存 RocksDB、存文件、存 S3……都有插件）。往这些 key `put` 的数据会被存下来。
- **Query（查询）**：任何节点可以对一个 key expression 发起 **query**（`session.get(...)`），Zenoh 会把**存储里匹配的历史数据**和**当前在线的 queryable 节点的应答**一起返回。这就是上一节说的"服务/请求-应答"语义，但更强——它统一了"查历史存储"和"问在线节点"。

```rust,ignore
// 查询：把 key expression "smart_driver/map/**" 下的数据都要回来
let replies = session.get("smart_driver/map/**").await.unwrap();
while let Ok(reply) = replies.recv_async().await {
    println!(">> {:?}", reply.result());
}
```

这对自动驾驶有很实在的用处：

- **地图/配置分发**：把高精地图切片存进 Zenoh storage，任何节点、任何时候 query 附近的地图，晚启动的节点也能立刻拿到——不用像 DDS 那样纠结 durability。
- **车-云数据回传的统一接口**：车上产生的数据 put 到某些 key，云端 storage 自动落盘；分析时用同一套 query 接口取。"在传"和"存下来"是同一套 key 空间，天然衔接第九部分的数据闭环。

> **中高级视角**：Zenoh 把 pub/sub、query、storage 统一在**同一个 key 空间**下，是它区别于纯 DDS 的最大设计哲学。DDS 世界里"实时消息"和"数据库"是两套东西；Zenoh 说它们只是同一份数据的"在传"和"在存"两个状态。这个统一在"车-边-云"贯通的数据闭环里价值巨大——你手边的 `inf/` 那种离线推理管线，消费的正是这样回传、落盘的数据，只是它站在闭环的另一端。

## 到底什么时候用 DDS，什么时候用 Zenoh

给你一张能直接拿去评审会用的决策表：

| 场景 / 诉求 | 更倾向 DDS | 更倾向 Zenoh |
|---|---|---|
| 车内局域网、节点数中等、要工业成熟度 | ✅ 默认稳妥 | 可以，但优势不明显 |
| 需要 ISO 26262 相关的认证积累 | ✅ DDS 积累更深 | 认证生态还年轻 |
| 跨网段：车-路-云、经 4G/5G 回传 | ❌ 多播假设不成立、较重 | ✅ 强项，router + 灵活拓扑 |
| 超大规模（数百节点、上千话题）发现开销 | 发现偏重 | ✅ 更轻，可收敛 |
| 团队是 Rust 栈、想要一等 Rust API | 只有 FFI 绑定 | ✅ 原生 Rust |
| 想统一实时消息 + 历史存储 + 查询 | 要另配数据库 | ✅ storage/query 内建 |
| 已有大量 ROS 2 + DDS 代码和经验 | ✅ 沿用即可 | 可用 `rmw_zenoh` 平滑迁移 |

一句话：**局域网内、要成熟和认证 → DDS（也是 ROS 2 现状默认）；跨云边端、大规模、Rust 栈、想统一存储查询 → Zenoh**。而且它们不是二选一的死对头——通过 `rmw_zenoh`，你可以让 ROS 2 应用层不变，底层从 DDS 换成 Zenoh，逐步迁移。

## 小结

- **DDS** 是 ROS 2 默认底座：去中心化自动发现（UDP 多播，无 master 单点）、QoS 极丰富、工业级成熟、对认证友好；但**发现在大规模下偏重、配置复杂、共享内存要费劲配**。
- **Zenoh** 是 **Rust 原生**的下一代：轻、拓扑灵活能跨云边端、内建 storage/query 与共享内存，被做成 ROS 2 的 `rmw_zenoh`；代价是生态与认证积累比 DDS 年轻。
- 我们用 `zenoh` crate 写了**纯 cargo、无需 ROS 工作空间**的完整发布/订阅：`zenoh::open` → `declare_publisher/subscriber` → `put`/`recv_async`，key 是层级化可通配的，payload 是字节（序列化格式自定）。
- **零拷贝共享内存**是大点云/图像实时流转的生死线：同机进程共享一块物理内存，只传引用不传数据，延迟与数据量几乎无关；前提是同机、要管好生命周期防撕裂、最好配零拷贝序列化格式。
- Zenoh 独有的 **storage/query** 把"在传的数据"和"存下来的数据"统一到同一 key 空间，天然衔接车-云数据闭环。
- 选型：**局域网 + 成熟/认证选 DDS，跨网段 + 大规模 + Rust 栈 + 统一存储选 Zenoh**，且可经 `rmw_zenoh` 平滑迁移。

下一节收尾第八部分：我们把注意力从"怎么传"转到"传什么"——消息定义即模块间的**契约**，以及 CDR、Protobuf、Cap'n Proto、FlatBuffers 这些序列化格式怎么选，还有用 `prost` 收发障碍物的完整 Rust 代码，以及为数据回放而生的 mcap 录制格式。
