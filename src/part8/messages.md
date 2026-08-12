# 8.4 消息定义与序列化

前两节我们反复回避一个问题：节点之间传的**字节到底长什么样**？8.2 里 ROS 2 帮我们生成了消息代码、钦定了 CDR 格式；8.3 里 Zenoh 干脆不管，我们随手用了 JSON。现在正面回答它——因为这是**模块间的契约（contract）**，是把第 1.3 节那五个模块的输入输出真正"钉死"下来的东西。

这一节讲：消息定义为什么是契约、IDL/msg 怎么写、几种主流序列化格式（CDR、Protobuf、Cap'n Proto、FlatBuffers、serde 生态）如何取舍、版本怎么演进而不把老节点搞崩、大消息（点云/图像）怎么处理，最后是录制格式 **mcap**——它把这张计算图的所有消息按时间戳存下来，是第九部分数据回放的基础。我们会用 `prost` 给出定义并收发一个障碍物消息的**完整 Rust 代码**。

## 消息即契约：把 1.3 的接口钉死

回想第 1.3 节：感知输出障碍物列表、预测输出多模态轨迹、规划输出轨迹点、控制输出指令。这些"输出→输入"的衔接处，就是模块间的**接口**。当模块变成独立进程后，接口不再是一个 Rust 函数签名（编译器能替你检查），而是**一段在网络上流动的字节**——没有编译器跨进程帮你检查了。

所以消息定义承担了原本编译器的活：它是一份**双方都同意、机器可读的契约**，规定"这个话题上的字节，必须能解释成这样一个结构"。契约的价值在于**解耦**——感知团队和规划团队只要都认这份 `Obstacle` 定义，各自用什么语言、怎么实现、什么时候发版，互不干扰。这也是 8.1 说的计算图"可局部替换"的根基：接口不变，节点随便换。

> **中高级视角**：把消息当契约，意味着**改消息是一件严肃的事**，和改一个公开 API 一样。加个字段、改个类型、删个字段，都可能悄悄搞崩线上某个你忘了的订阅者。成熟团队会把消息定义单独放一个仓库（像我们的 `smart_driver_msgs`）、做评审、做版本管理——因为它是几十个节点共同依赖的地基。

## IDL / msg：用中立语言描述结构

契约不能用某一种语言的结构体写（否则别的语言的节点读不懂），要用一种**中立的接口描述语言（IDL，Interface Definition Language）**。8.2 里的 `.msg` 就是 ROS 2 的 IDL；Protobuf 有 `.proto`；Cap'n Proto 有 `.capnp`。它们长得都差不多——声明字段名、类型、顺序：

```text
# ROS 2 .msg（8.2 见过）
uint32     id
float64[3] position
float32    confidence
```

```protobuf
// Protobuf .proto
message Obstacle {
  uint32 id = 1;              // 注意每个字段有个"字段号"
  repeated double position = 2;
  float confidence = 3;
}
```

一个**代码生成器（code generator）**吃掉 IDL，为每种目标语言吐出对应的结构体/类，以及"如何把它变成字节 / 从字节变回来"的序列化代码。你写 IDL 一次，C++/Python/Rust 各得一套一致的类型——这就是跨语言互通的机制，8.2 的 `rosidl_generator_rs`、下面要用的 `prost` 都是这类生成器。

## 序列化格式横向对比

**序列化（serialization）** = 把内存里的结构变成一串可传输/可存储的字节；**反序列化**是逆操作。格式的选择直接影响延迟、带宽、兼容性，是中高级工程师要能张口就答的取舍。

| 格式 | 编码 | 速度 | 体积 | 零拷贝 | 模式演进 | 自描述 | 智驾里的位置 |
|---|---|---|---|---|---|---|---|
| **JSON** | 文本 | 慢 | 大 | 否 | 灵活 | 是 | 调试、配置、Web 接口；**不上高频数据流** |
| **ROS CDR** | 二进制 | 快 | 小 | 否 | 弱（靠约定） | 否 | ROS 2 的线上格式，DDS 标准 |
| **Protobuf** | 二进制 | 快 | 小 | 否 | **强（字段号）** | 否 | 跨服务/跨语言事实标准，Apollo 在用 |
| **Cap'n Proto** | 二进制 | 极快 | 中 | **是** | 强 | 否 | 追求极致延迟、内存即传输 |
| **FlatBuffers** | 二进制 | 极快 | 中 | **是** | 强 | 否 | 大消息零拷贝读，游戏/车机常用 |

逐个说清它们的性格：

- **JSON**：人能读、自描述、改起来随便，但**文本编码又慢又占地方**（一个 `f64` 要十几个字符）。它属于"调试和低频配置"，8.3 里我们用它纯粹是为了让例子简单。**任何高频数据流用 JSON 都是错的**。

- **CDR（Common Data Representation）**：DDS/ROS 2 的线上格式。紧凑的二进制、编解码快。它**不自描述**——字节里没有字段名和类型信息，全靠收发双方共享同一份 `.msg` 定义来解释。所以 8.2 那句"序列化格式才是真正的契约"：Rust 节点和 C++ 节点内存布局不同，但都按 CDR 规则编码，就能互通。

- **Protobuf（Protocol Buffers）**：Google 出品，跨语言序列化的事实标准，百度 Apollo 大量使用。杀手锏是**用字段号（tag）而非字段顺序来标识字段**——这让它的**版本演进特别稳健**（下节详述）。Rust 侧有极好的 `prost` crate，下面就用它。

- **Cap'n Proto**：号称"比 Protobuf 快无限倍"，因为它**没有编解码步骤**——它的内存布局**就是**传输布局。你拿到字节，直接当结构体读，不用 parse。这正是 8.3 说的**零拷贝**的序列化侧。

- **FlatBuffers**：思路和 Cap'n Proto 类似——**零拷贝读取**，拿到 buffer 不解析就能按需访问任一字段。特别适合**大消息、只读部分字段**的场景（比如订阅者只想看点云消息头里的时间戳，不想 parse 全部 12 万个点）。

- **serde 生态**：Rust 的 `serde` 是一套序列化**框架**，不是具体格式。给结构体 `#[derive(Serialize, Deserialize)]`，就能配上各种后端：`serde_json`（JSON）、`bincode`（紧凑二进制）、`postcard`（为嵌入式优化的紧凑格式）、`ciborium`（CBOR）等。**纯 Rust、不跨语言的内部通信**，`serde + bincode/postcard` 是最省事的组合——8.3 那个例子就可以把 `serde_json` 换成 `bincode` 立刻变快变小。

选型口诀：

- **在 ROS 2 里** → 你没得选，就是 CDR（自动的），你只管写 `.msg`。
- **跨语言、跨服务、要长期演进** → **Protobuf**（`prost`）。
- **同机大消息、追求极致零拷贝** → **Cap'n Proto / FlatBuffers**，配 8.3 的共享内存。
- **纯 Rust 内部、图省事** → **serde + bincode/postcard**。
- **JSON** → 只在调试、配置、和外部 Web 系统对接时用。

## 用 prost 定义并收发障碍物（完整 Rust 代码）

`prost` 是 Rust 里用 Protobuf 的主流方式：它把 `.proto` 编译成地道的 Rust 结构体（普通 `struct` + `#[derive]`，不是丑陋的生成代码），序列化走 `bytes`。我们把 8.3 那个障碍物换成 Protobuf 编码。

项目 `prost_demo`，`Cargo.toml`：

```toml
[package]
name = "prost_demo"
version = "0.1.0"
edition = "2021"

[dependencies]
prost = "0.13"        # Protobuf 运行时
bytes = "1"           # prost 用它做零拷贝的字节缓冲

[build-dependencies]
prost-build = "0.13"  # 在 build.rs 里把 .proto 编成 Rust
```

`.proto` 文件放 `proto/obstacle.proto`：

```protobuf
syntax = "proto3";
package smart_driver;

// 障碍物类别用 enum，比裸 uint8 更自描述
enum ObstacleClass {
  UNKNOWN = 0;      // proto3 要求第一个枚举值为 0
  VEHICLE = 1;
  PEDESTRIAN = 2;
  CYCLIST = 3;
}

// 对应第 1.3 节的 Obstacle。字段号（= 后面的数字）一旦分配，永不复用。
message Obstacle {
  uint32 id = 1;
  ObstacleClass obstacle_class = 2;
  repeated double position = 3;   // x, y, z
  repeated double velocity = 4;
  repeated double size = 5;
  double heading = 6;
  float confidence = 7;
}

// 一帧感知输出
message ObstacleArray {
  double stamp = 1;               // 时间戳（秒）
  string frame_id = 2;            // 坐标系名，如 "ego"
  repeated Obstacle obstacles = 3;
}
```

`build.rs`（放项目根）——构建时自动生成 Rust 代码：

```rust,ignore
fn main() {
    prost_build::compile_protos(&["proto/obstacle.proto"], &["proto/"])
        .expect("编译 .proto 失败");
}
```

`src/main.rs`——在一个进程里演示"发送方编码 → 字节 →（这里就代表在网络上传输）→ 接收方解码 → 处理"，把序列化闭环讲透。真实系统里这两半分别在 8.2 的发布回调、订阅回调，或 8.3 的 `publisher.put` / `subscriber.recv_async` 里：

```rust,ignore
use prost::Message; // 提供 encode_to_vec / decode

// 引入 prost 生成的模块。prost_build 默认按 proto 的 package 名建模块。
pub mod smart_driver {
    include!(concat!(env!("OUT_DIR"), "/smart_driver.rs"));
}
use smart_driver::{obstacle_class, Obstacle, ObstacleArray, ObstacleClass};

/// 发送方：造一帧障碍物并编码成 Protobuf 字节。
fn encode_frame(seq: u32) -> Vec<u8> {
    let dt = 0.1_f64;
    let lateral = -1.5 + 0.3 * (seq as f64) * dt;

    let pedestrian = Obstacle {
        id: 42,
        // prost 把 proto enum 生成为 i32 字段；用生成的枚举取值更安全。
        obstacle_class: ObstacleClass::Pedestrian as i32,
        position: vec![30.0, lateral, 0.0],
        velocity: vec![0.0, 0.3, 0.0],
        size: vec![0.5, 0.5, 1.7],
        heading: std::f64::consts::FRAC_PI_2,
        confidence: 0.92,
    };

    let frame = ObstacleArray {
        stamp: seq as f64 * dt,
        frame_id: "ego".to_string(),
        obstacles: vec![pedestrian],
    };

    // 核心：编码成字节。这就是会被 put 到话题上的东西。
    frame.encode_to_vec()
}

/// 接收方：从字节解码回 ObstacleArray 并做风险判断。
fn decode_and_process(bytes: &[u8]) -> Result<(), prost::DecodeError> {
    let frame = ObstacleArray::decode(bytes)?; // 从字节还原结构
    println!(
        "[recv] t={:.1}s frame={} 共 {} 个障碍物",
        frame.stamp,
        frame.frame_id,
        frame.obstacles.len()
    );

    for ob in &frame.obstacles {
        // 把 i32 转回枚举，proto3 里未知值会落到 Unknown 之外，需处理。
        let class = obstacle_class::from_i32_name(ob.obstacle_class);
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
                "  ⚠ {:?}#{} 前方 {:.1}m 横穿，约 {:.1}s 到车道中心 -> 减速让行",
                class, ob.id, ahead, ttc
            );
        } else {
            println!("  · {:?}#{} 前方 {:.1}m，暂无风险", class, ob.id, ahead);
        }
    }
    Ok(())
}

// 辅助：把 i32 映射回可读枚举名（生成的代码提供 ObstacleClass::try_from）
mod obstacle_class {
    use super::ObstacleClass;
    pub fn from_i32_name(v: i32) -> ObstacleClass {
        ObstacleClass::try_from(v).unwrap_or(ObstacleClass::Unknown)
    }
}

fn main() {
    // 模拟三帧：编码 -> （网络/共享内存传输）-> 解码处理
    for seq in 0..3 {
        let bytes = encode_frame(seq);
        println!(
            "[send] 第 {seq} 帧编码为 {} 字节 protobuf",
            bytes.len()
        );
        // 这里 bytes 就是会在 8.2 的 publisher.publish 或
        // 8.3 的 publisher.put 里被发送的负载。
        decode_and_process(&bytes).expect("解码失败");
        println!();
    }
}
```

`cargo run` 会看到每帧被编码成**几十字节**的紧凑二进制（对比同样内容的 JSON 会大好几倍），解码后行人 `y` 逐帧逼近车道、`ttc` 递减——又是第 1.3 节那个场景，这次以 Protobuf 契约的形态。

把这段接进真实中间件很直接：`encode_to_vec()` 的结果就是 8.3 里 `publisher.put(bytes)` 的参数，`ObstacleArray::decode(sample.payload().to_bytes())` 就是订阅回调里的第一句。**Protobuf 负责"结构↔字节"，中间件负责"字节从这里到那里"，两件事正交。**

## 版本兼容与演进：别把老节点搞崩

契约会变——需求变了，`Obstacle` 要加个"是否被遮挡"的字段。问题是：**新节点发的消息，老节点还能读吗？老节点发的，新节点能读吗？** 这就是**前向/后向兼容（forward/backward compatibility）**。

Protobuf 在这件事上是标杆，规则简单到能背下来：

- **加字段**：给个**新的字段号**。老节点解码时遇到不认识的字段号，**直接跳过**，照常工作（后向兼容）；新节点读老消息时，缺的字段取默认值（前向兼容）。**所以加字段是安全的。**
- **删字段**：**永远不要复用它的字段号**。把字段标记为 `reserved`，防止将来有人拿这个号做别的、和老数据打架。
- **改类型/改字段号**：**危险**，等于毁约。老新节点会用不同方式解释同一段字节，静默出错。要改就当成"加新字段 + 弃用旧字段"。

```protobuf
message Obstacle {
  // ... 原有字段 ...
  bool occluded = 8;          // ✅ 安全：新字段号，老节点忽略它
  reserved 9;                 // 曾经用过、已删除的字段号，永久占位
  reserved "old_field_name";
}
```

对比一下：**CDR（ROS 2）没有字段号**，靠字段顺序和 `.msg` 的严格一致来解释字节。所以在 ROS 2 里改消息**更脆**——加个字段就可能让老节点错位解析。ROS 2 生态因此对"消息包版本"管理很谨慎。这正是 Apollo 选 Protobuf、以及很多"要长期演进"的系统偏爱 Protobuf 的原因。

> **面试题**：为什么 Protobuf 加字段是安全的，而在 ROS 2（CDR）里加字段可能搞崩老节点？
> **答**：Protobuf 用**字段号**标识每个字段，编码里带着号，老节点遇到不认识的号就跳过、缺失字段用默认值，所以新老可以共存。CDR **不带字段标识**，纯按 `.msg` 里字段的顺序和类型紧密排列字节；加一个字段会改变后续字节的偏移，老节点仍按旧布局解析，就会**错位**读到垃圾。前者为演进而设计，后者为紧凑而牺牲了演进弹性。

## 大消息：点云和图像怎么办

障碍物列表才几十字节，但点云 2 MB、图像 6 MB——大消息有它自己的一套讲究，把前几节的线索收在一起：

1. **别把大数据当普通消息反复序列化**。一帧点云 12 万个 `(x,y,z,intensity)`，用 Protobuf 的 `repeated Point` 去编码，光遍历打包就很慢。常见做法是**把点云打包成一段紧凑的字节 blob**（固定布局的 `Vec<u8>`，如 ROS 的 `PointCloud2` 就是"头 + 一大块原始字节"），消息里只放这块 blob 和描述它布局的元信息（每个点几字节、字段偏移）。解析时按元信息 `reinterpret`，不逐点反序列化。

2. **同机就上零拷贝共享内存**（8.3）。大消息是零拷贝收益最大的地方。配合 **FlatBuffers/Cap'n Proto** 这类"内存即传输"的格式，订阅者拿到共享内存指针**直接当结构体读**，全程零拷贝、零解析。这是"点云 30 Hz 实时"的标准解法。

3. **消息里带 header**（时间戳 + frame_id）。大消息尤其要能只读 header 判断"这帧要不要处理"，而不解开整块数据——FlatBuffers 的按需访问在这里发光。

4. **能不发就不发**。如果多个节点在同机、且都要原始点云，考虑让它们共享同一块内存而不是各订阅一份。传输最快的数据是**没被传输的数据**。

## 录制格式：rosbag 与 mcap，接上数据闭环

计算图的最后一块拼图：既然所有模块只通过话题上的消息通信，那把**所有话题的消息连同时间戳录下来**，就等于录下了整个系统在某段时间的完整"经历"。事后离线重放这些消息，就能**精确复现**一次路测——这是仿真、测试、数据闭环的基石（第九部分主角）。

- **rosbag**：ROS 传统的录制格式（ROS 2 是 `rosbag2`）。`ros2 bag record /obstacles /ego_pose ...` 就把这些话题的消息全存下来，`ros2 bag play` 重放。它和 ROS 深度绑定。

- **mcap**：一个**更现代、自描述、与框架无关**的录制容器格式（由 Foxglove 推动，已成 ROS 2 的推荐存储格式）。它的好处：不绑定 ROS（Zenoh、自研中间件的数据也能存）、内建索引（能快速跳到某个时间点，不用从头扫）、每条消息带 schema（自描述，多年后还能解开）、对大消息友好。**而且它有 Rust 库**（`mcap` crate），和这本书的技术栈天然契合。

用 `mcap` crate 读一个录制文件、遍历消息的骨架（真实项目里你会拿这些字节喂给 prost/CDR 解码）：

```rust,ignore
// Cargo.toml: mcap = "0.9",  memmap2 = "0.9"
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let data = fs::read("drive_2026_08_12.mcap")?;
    // mcap::MessageStream 按写入顺序（或时间）遍历所有消息
    for message in mcap::MessageStream::new(&data)? {
        let msg = message?;
        // channel 描述这条消息属于哪个话题、用什么 schema/编码
        println!(
            "t={} 话题={} 负载={} 字节",
            msg.log_time,
            msg.channel.topic,
            msg.data.len()
        );
        // 若 channel.message_encoding 是 "protobuf"，就用上面的
        // ObstacleArray::decode(&msg.data) 把它还原成结构体；
        // 若是 "cdr"，则用 CDR 解码。schema 名告诉你是哪种消息。
    }
    Ok(())
}
```

看出闭环了吗：**在线**时你的 Rust 节点用 prost/CDR 编码消息、经 ROS 2 或 Zenoh 发出；一个录制节点把它们连同时间戳、schema 写进 mcap；**离线**时你用 `mcap` crate 读回来、用同一份契约解码、喂给算法重跑——这正是你手边 `inf/` 那类离线推理管线做的事的输入端。**消息契约在在线和离线两侧是同一份**，这就是为什么 8.1 说"在线是计算图、离线是数据管线，共享同一套节点+消息的世界观"。

## 小结

- 消息定义是模块间的**契约**，把第 1.3 节各模块的 IO 从"函数签名"升级为"跨进程的字节约定"；改它是严肃的事，应单独仓库 + 评审 + 版本管理。
- 用中立的 **IDL/msg**（`.msg`/`.proto`/`.capnp`）描述结构，代码生成器为各语言产出一致类型——这是跨语言互通的机制。
- 序列化格式取舍：**ROS 里用 CDR**（自动）、**跨语言长期演进用 Protobuf（prost）**、**同机大消息极致零拷贝用 Cap'n Proto/FlatBuffers**、**纯 Rust 内部用 serde+bincode/postcard**、**JSON 只配调试**。
- 我们用 `prost` 写了 `.proto` 定义 + `build.rs` 生成 + 编码/解码/处理的**完整 Rust 代码**，`encode_to_vec()`/`decode()` 正好接 8.2 的 publish / 8.3 的 put。
- **版本演进**：Protobuf 靠字段号，加字段安全、删字段要 `reserved`、改类型危险；CDR 无字段号更脆——这是 Protobuf 受青睐的核心原因。
- **大消息**（点云/图像）：打包成 blob + 元信息、同机上零拷贝共享内存 + FlatBuffers、带 header、能不发就不发。
- **录制格式**：`rosbag2` 绑 ROS，**mcap** 自描述、跨框架、有索引、**有 Rust 库**，把计算图的消息连同时间戳/schema 存下，在线编码与离线解码共享同一份契约——直通第九部分的仿真与回放。

第八部分到此完整：从中间件全景（8.1）、到 ROS 2 + rclrs 写节点（8.2）、到 Zenoh/DDS 与零拷贝（8.3）、到消息契约与序列化（8.4），你已经能把第一部分那张"并发的节点图"用 Rust 真正搭起来、让它们高效可靠地对话、并把对话录下来供日后回放。下一部分，我们就走进那个"回放"的世界——仿真、测试与安全。
