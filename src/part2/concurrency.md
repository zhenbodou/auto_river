# 2.5 并发与异步：线程、channel、tokio

第 1.3 节结尾我埋了个伏笔：真实的自动驾驶系统**不是一个函数顺序调用**，而是"并发的、消息驱动的、各跑各频率的节点图"——感知 10 Hz、控制 100 Hz，它们同时运行、互相订阅消息。这一章就是兑现那个伏笔的地方，也是本部分第二个硬骨头。

Rust 在并发上有个响亮的口号：**无畏并发（fearless concurrency）**。它的意思很具体——上一章讲的所有权和借用规则，**同样管着线程之间的数据共享**，于是数据竞争在**编译期**就被拒绝了（第 1.4 节说过）。你依然可能写出死锁或逻辑错误，但那种"多线程偶发翻车、调三天查不出"的数据竞争 bug，Rust 从根上帮你消灭了。这一章我们把这套东西用到智驾场景里，最后从**实时性**的角度审视那些能真正要命的坑。

## 线程：最朴素的并发

Rust 标准库的线程就是操作系统线程（OS thread），和 C++ 的 `std::thread` 一个层次。启动一个：

```rust,ignore
use std::thread;

fn main() {
    let handle = thread::spawn(|| {
        // 在新线程里跑
        heavy_pointcloud_processing();
    });

    // 主线程继续干别的
    do_other_work();

    handle.join().unwrap(); // 等那个线程结束
}
```

`thread::spawn` 接一个闭包，返回一个 `JoinHandle`。`join()` 阻塞等它结束。到这为止和别的语言没啥两样。有意思的是当你想在线程间**共享数据**时，Rust 的规则开始发威。

## 和编译器搏斗：跨线程共享障碍物列表

假设你想启动几个线程，各自读同一份障碍物列表。天真的写法：

```rust,ignore
fn main() {
    let obstacles = get_obstacles(); // Vec<Obstacle>
    let handle = thread::spawn(|| {
        println!("障碍物数量: {}", obstacles.len()); // ❌ 借用了外部的 obstacles
    });
    handle.join().unwrap();
}
```

编译器报错：`closure may outlive the current function, but it borrows obstacles`。**为什么？** 编译器无法证明 `obstacles` 会活得比这个线程久——线程可能在 `main` 结束后还在跑（虽然这里有 `join`，但编译器不做这种跨函数推理）。如果 `obstacles` 先被释放、线程还在用它，就是悬垂——正是第 2.2 节那个问题的多线程版。

`move` 关键字把所有权**移进**闭包能解决单线程独占的情况：

```rust,ignore
let handle = thread::spawn(move || {
    println!("障碍物数量: {}", obstacles.len()); // ✅ obstacles 被移进来，归线程所有
});
```

但如果**多个线程都要读**同一份数据呢？所有权只能给一个线程。这就需要 `Arc`。

## Arc：多个线程共享只读数据

**`Arc<T>`**（Atomically Reference Counted，原子引用计数）是一个"可以被多个所有者共享"的智能指针。它内部维护一个原子计数器，记录有多少个 `Arc` 指向同一份数据；每 `clone` 一个 `Arc`，计数 +1；每个 `Arc` 离开作用域，计数 -1；归零时才真正释放数据。**注意 `clone` 一个 `Arc` 只是拷贝那个指针和加个计数，绝不拷贝底层那 2 MB 数据**——这点对点云至关重要。

```rust,ignore
use std::sync::Arc;
use std::thread;

fn main() {
    // 一帧点云，包成 Arc，准备分给多个线程只读
    let cloud = Arc::new(get_pointcloud()); // Arc<PointCloud>

    let mut handles = vec![];
    for region in 0..4 {
        let cloud = Arc::clone(&cloud); // 只是加引用计数，2 MB 数据不拷贝
        let h = thread::spawn(move || {
            // 每个线程处理点云的一个区域，只读
            let n = count_points_in_region(&cloud, region);
            println!("区域 {} 有 {} 个点", region, n);
        });
        handles.push(h);
    }
    for h in handles {
        h.join().unwrap();
    }
} // 所有 Arc 都 drop 后，点云在此释放
```

这就是"多线程分区并行处理一帧点云"的骨架：数据只有一份，靠 `Arc` 共享，零拷贝。**为什么是 `Arc` 而不是 `Rc`？** Rust 还有个非原子的 `Rc<T>`，计数更新更快但**不是线程安全的**——编译器不允许你把 `Rc` 送进线程。这又是所有权系统在替你把关：想跨线程，就得用线程安全的 `Arc`，编译期强制，你想用错都用不了。

## Mutex 与 RwLock：共享可变数据

`Arc` 只解决了共享**只读**数据。可如果多个线程要**修改**同一份数据呢？比如一个"全局障碍物地图"，感知线程往里写、规划线程从里读。这违反了第 2.2 节"共享 XOR 可变"——多个线程要写同一块内存。

Rust 的答案是**锁**，而且它的锁设计比 C++ 高明：**锁把数据包在里面，你不拿到锁就碰不到数据**。这叫"锁保护数据"而非"锁和数据靠约定关联"。

**`Mutex<T>`**（互斥锁）——任意时刻只有一个线程能访问里面的数据：

```rust,ignore
use std::sync::{Arc, Mutex};

fn main() {
    // 共享的、可变的障碍物地图
    let world = Arc::new(Mutex::new(Vec::<Obstacle>::new()));

    // 感知线程：往里写
    let world_w = Arc::clone(&world);
    let perception = thread::spawn(move || {
        loop {
            let new_obstacles = perceive();
            let mut guard = world_w.lock().unwrap(); // 拿锁，得到可变访问
            *guard = new_obstacles;                   // 更新
            // guard 离开作用域时自动解锁——不可能忘记 unlock！
        }
    });

    // 规划线程：读
    let world_r = Arc::clone(&world);
    let planning = thread::spawn(move || {
        loop {
            let obstacles = {
                let guard = world_r.lock().unwrap();
                guard.clone() // 快速拷一份出来，尽早释放锁（见下文实时性讨论）
            };
            plan_with(&obstacles);
        }
    });
    // ...
}
```

注意 `Arc<Mutex<T>>` 这个组合——`Arc` 负责"多个线程共享这个锁"，`Mutex` 负责"同一时刻只有一个线程能改数据"。这是 Rust 里共享可变状态的**标准搭配**，你会见到无数次。

还有两个 Rust 锁的优点值得点明：

1. **锁自动释放**。`lock()` 返回一个 `MutexGuard`，它离开作用域时自动解锁。C++ 里忘写 `unlock`（尤其是在提前 return 或异常路径上）导致死锁的经典 bug，在 Rust 里不存在——这是 RAII 的又一次胜利。
2. **`.lock()` 返回 `Result`**。为什么？因为如果某个持锁的线程 panic 了，锁会进入"中毒（poisoned）"状态，提醒你"被锁保护的数据可能处于不一致状态"。这个设计逼你直面"持锁线程崩了怎么办"，而不是假装无事发生。

**`RwLock<T>`**（读写锁）——当"读多写少"时用它更好：允许**多个读者同时读**，或**一个写者独占写**。这正好是第 2.2 节"共享 XOR 可变"规则的运行时版本！

```rust,ignore
use std::sync::RwLock;

let map = Arc::new(RwLock::new(HdMap::load()));

// 多个规划/预测线程可以同时读地图
let r = map.read().unwrap();  // 共享读锁，多个线程能同时持有
use_map(&r);

// 地图更新线程独占写
let mut w = map.write().unwrap(); // 独占写锁
w.update_lane_info();
```

高精地图这种"加载后极少变、被无数模块高频读"的数据，用 `RwLock` 比 `Mutex` 好——`Mutex` 会让读者之间也互斥排队，白白串行化。

## channel：更好的并发模型

锁能用，但共享可变状态是并发 bug 的重灾区（死锁、竞态逻辑）。有一句在并发编程里流传很广的话：

> **不要通过共享内存来通信，而要通过通信来共享内存。**

意思是：与其让多个线程抢一块共享内存，不如让它们**各自拥有自己的数据，通过传递消息来协作**。这就是 **channel（通道）** 模型——它天然契合"感知/规划/控制是各跑各的、通过消息通信的节点"这个心智模型。

标准库的 channel 是 **mpsc**（multi-producer, single-consumer，多生产者单消费者）：

```rust,ignore
use std::sync::mpsc;
use std::thread;

fn main() {
    let (tx, rx) = mpsc::channel::<Vec<Obstacle>>(); // tx 发送端，rx 接收端

    // 感知节点：不停产出障碍物列表，通过 channel 发出去
    thread::spawn(move || {
        loop {
            let obstacles = perceive();
            if tx.send(obstacles).is_err() {
                break; // 接收端没了，退出
            }
        }
    });

    // 规划节点：从 channel 收，收到就规划
    for obstacles in rx {  // rx 可以直接迭代，收一个处理一个
        let trajectory = plan(&obstacles);
        execute(&trajectory);
    }
}
```

看这段代码多干净：感知线程**拥有**它产出的障碍物列表，用 `send` 把**所有权转移**给规划线程——没有共享、没有锁、没有数据竞争的可能。所有权系统和 channel 在这里配合得天衣无缝：`send(obstacles)` 之后，感知线程就再也碰不到 `obstacles` 了（它被移走了），编译器保证。**"发出去的数据我不再动"这个并发纪律，被所有权变成了编译期强制。**

### crossbeam：标准库 channel 的增强版

标准库的 mpsc 只支持"单消费者"，且性能一般。真实项目常用 **`crossbeam`** crate，它提供：

- **mpmc**（multi-producer multi-consumer）channel——多个消费者也行；
- **有界 channel**（`bounded(n)`）——队列满了发送方会阻塞，天然实现**背压（backpressure）**；
- 更好的性能和 `select!`（同时等多个 channel）。

```toml
[dependencies]
crossbeam = "0.8"
```

```rust,ignore
use crossbeam::channel::{bounded, select};

// 有界 channel：容量 4。满了发送方阻塞——防止感知产出太快把内存撑爆
let (tx, rx) = bounded::<Vec<Obstacle>>(4);
```

> **中高级视角**：**有界 channel 是实时系统里的关键设计**。用无界队列，如果生产者（比如高频的雷达）比消费者（比如慢的规划）快，队列会无限增长——内存暴涨、延迟越来越大（你处理的永远是几秒前的旧数据），最后 OOM 崩溃。有界 channel 强迫你面对"处理不过来怎么办"：是阻塞上游、还是丢弃旧帧（对实时系统，处理最新帧往往比处理完所有帧更重要）？这个决策必须显式做，而有界 channel 逼你做。这是初级和资深并发工程师的一个分野。

## 把流水线搭成并发节点图

把上面的东西组装起来，就是第 1.3 节说的那个"并发节点图"的骨架。感知、规划、控制各是一个线程，用 channel 串起来：

```rust,ignore
use crossbeam::channel::bounded;
use std::thread;

fn run_pipeline() {
    // 各段之间用有界 channel 连接，容量都设小，保证处理的是新鲜数据
    let (obs_tx, obs_rx) = bounded::<Vec<Obstacle>>(2);
    let (traj_tx, traj_rx) = bounded::<Trajectory>(2);

    // 感知节点：10 Hz
    thread::spawn(move || loop {
        let obstacles = perceive();
        let _ = obs_tx.send(obstacles); // channel 满就阻塞（背压）
    });

    // 规划节点：收障碍物，出轨迹
    thread::spawn(move || {
        for obstacles in obs_rx {
            let traj = plan(&obstacles);
            let _ = traj_tx.send(traj);
        }
    });

    // 控制节点：100 Hz，比规划快得多，所以它不能傻等新轨迹
    thread::spawn(move || {
        let mut current_traj: Option<Trajectory> = None;
        loop {
            // 非阻塞地看看有没有新轨迹，没有就继续用旧的
            if let Ok(traj) = traj_rx.try_recv() {
                current_traj = Some(traj);
            }
            if let Some(traj) = &current_traj {
                let cmd = control_tick(traj); // 用上一章那个不会 panic 的版本
                send_to_chassis(cmd);
            }
            spin_until_next_100hz_tick();
        }
    }).join().unwrap();
}
```

**注意控制节点用 `try_recv`（非阻塞）而不是 `recv`（阻塞）**。这是不同频率节点协作的关键：控制 100 Hz 比规划快，它不能停下来等新轨迹——它拿最新可用的轨迹，没新的就沿用旧的（轨迹本身就是一条未来几秒的曲线，本就设计成"能用一小会儿"）。**理解"每个节点按自己的节奏跑、用最新可用数据"是理解真实自动驾驶系统运行方式的核心。** 第八部分我们会用真正的中间件（ROS 2 / Zenoh）把这套东西搭得更工业化，但底层思想就是这里的 channel。

## async/await 与 tokio：另一种并发

到这你可能会问：既然有线程，为什么 Rust 还有 `async`/`await`？它们不是一回事，解决的是不同问题。

**线程适合 CPU 密集（CPU-bound）任务**——点云处理、神经网络推理、路径搜索，这些在**烧 CPU**。你有 8 个核，就开差不多 8 个线程把核喂满。开成百上千个线程没意义（线程切换本身有开销，且每个线程要栈内存）。

**async 适合 IO 密集（IO-bound）任务**——网络请求、读传感器驱动、等硬件响应。这些任务大部分时间在**等**（等网卡、等磁盘、等设备），CPU 是空闲的。如果每个"等待"都占一个线程，等 1000 个连接就要 1000 个线程，浪费。`async` 让你用**少量线程**（比如就 CPU 核数那么多）驱动**成千上万个并发任务**——某个任务一旦开始"等"，就把线程让给别的任务，等的事情好了再回来。

`async` 语法：

```rust,ignore
// async fn 返回一个 Future（“未来会产出一个值”的东西），调用它不会立刻执行
async fn fetch_hd_map_tile(x: i32, y: i32) -> Result<MapTile, NetError> {
    let resp = http_get(&format!("/tiles/{x}/{y}")).await?; // .await 在这里“让出”
    let tile = parse_tile(resp).await?;
    Ok(tile)
}
```

`.await` 是关键：它标记一个"可能要等"的点。执行到 `.await` 且数据还没好时，当前任务**挂起**、把线程让给别的任务，而不是傻等阻塞。

Rust 语言本身只提供 `async`/`await` 语法，**真正驱动这些任务运行的"运行时（runtime）"要靠库**，最主流的是 **`tokio`**：

```toml
[dependencies]
tokio = { version = "1", features = ["full"] }
```

```rust,ignore
#[tokio::main] // 这个宏把 main 变成异步运行时的入口
async fn main() {
    // 并发地拉取 4 个地图瓦片——它们同时在“等网络”，不是串行
    let (t1, t2, t3, t4) = tokio::join!(
        fetch_hd_map_tile(0, 0),
        fetch_hd_map_tile(0, 1),
        fetch_hd_map_tile(1, 0),
        fetch_hd_map_tile(1, 1),
    );
    // 四个请求几乎同时完成，总耗时约等于最慢的那个，而不是四个之和
}
```

### 智驾里到底什么时候用 async、什么时候用线程

这是个高频的实际决策，给你一张清楚的判据：

| 场景 | 用什么 | 为什么 |
|------|--------|--------|
| 点云滤波、聚类、体素化 | 线程 + `rayon` | CPU 密集，要吃满多核 |
| 神经网络推理 | 线程（推理库内部并行） | CPU/GPU 密集 |
| 路径搜索、优化求解 | 线程 | CPU 密集 |
| 读多路网络传感器/V2X | async + tokio | IO 密集，大量并发等待 |
| 与云端通信、OTA、日志上传 | async + tokio | IO 密集 |
| 传感器驱动的异步读取 | async 或专用线程 | 看驱动接口形态 |

**经验法则：算力活儿用线程（配合 `rayon` 做数据并行，见第 2.6 节），等待活儿用 async。** 一个真实系统里两者常常并存——感知/规划/控制的重计算跑在线程池上，而与外界通信的部分（网络、云、诊断上报）跑在 tokio 上。别犯"什么都往 async 里塞"或"什么都开线程"的教条错误。

> **面试题**：为什么不能在 async 任务里直接跑一个耗时几十毫秒的 CPU 密集计算？
> **答**：因为 async 任务共享少数几个执行线程，靠"遇到 `.await` 就让出"来协作。一个不 `.await`、闷头算几十毫秒的任务会**霸占**执行线程，让同一线程上的其它所有异步任务全部饿死（starvation），破坏整个运行时的响应性。正确做法是把这种计算用 `tokio::task::spawn_blocking` 丢到专门的阻塞线程池，或干脆用独立的 CPU 线程池（rayon）处理。**"async 里不能有长时间不让出的计算"是用好 tokio 的第一纪律。**

## 实时性视角：那些能要命的坑

前面讲的都是"怎么让它并发起来"。但在车上，光并发正确还不够——你还要它**及时**。这一节讲三个从实时性（real-time）角度看能真正要命的东西。它们在服务器上无所谓，在车上是安全问题。

### 1. 锁竞争（lock contention）

`Mutex` 保证了正确性，但一个线程持锁时，其它想拿锁的线程**全在阻塞排队**。如果你在持锁期间干了耗时的活儿——尤其别在持锁时做重计算或 IO——那些等锁的实时任务就被拖住，控制指令可能因此延迟。**纪律：锁的临界区要尽可能短**。看前面规划线程那段代码，它 `guard.clone()` 快速拷一份数据出来就立刻放锁，把耗时的 `plan_with` 放在锁外面做。这个"进去拷一份、马上出来、在外面慢慢算"的模式是实时代码里的常见手法。

### 2. 优先级反转（priority inversion）

这是实时系统的经典杀手，值得单独讲。设想：一个**高优先级**的控制线程要拿锁，但锁被一个**低优先级**的日志线程持有；偏偏这个低优先级线程又被一个**中优先级**线程抢占了 CPU 迟迟跑不完。结果：高优先级的控制线程,只能干等那个低优先级线程释放锁，而后者被中优先级线程压着。**高优先级任务被间接地卡在了低优先级任务后面**——优先级被"反转"了。历史上著名的火星探路者号（Mars Pathfinder）故障就是这个原因。

对策是操作系统层面的**优先级继承（priority inheritance）**协议（持锁的低优先级线程临时"继承"等待者的高优先级），以及在设计上——**让安全攸关的实时控制回路尽量不和低优先级任务共享锁**。这也是为什么很多实时系统偏爱 channel 而非共享锁：消息传递没有"持有资源不放"的问题。

### 3. 分配抖动（allocation jitter）

这是最隐蔽、也最 Rust 相关的一个。默认的堆分配（`Vec::new`、`Box::new`、`String`，以及任何隐式分配）耗时是**不确定的**——大多数时候几十纳秒，但偶尔操作系统要向内核申请新内存页、或分配器要整理内存，某一次分配可能突然耗时几十微秒甚至更多。对 100 Hz(10 ms 周期）的控制回路，这种偶发的"卡一下"（jitter，抖动）可能就让你错过一个控制周期。**GC 语言的停顿是这个问题的极端放大版，这也是第 1.4 节说 GC 语言出局的原因**；但即使没有 GC，malloc 本身也有抖动。

对策：**在实时热路径上避免动态分配**。具体做法——

- **预分配**：启动时把缓冲区（如点云 buffer、障碍物 Vec）一次性 `Vec::with_capacity(n)` 分配好，之后循环里**复用**（用 `.clear()` 清空后重填，而不是每帧 `Vec::new()`）。
- **对象池**：维护一组可复用的缓冲区，用完还回去，避免反复分配释放。
- **无堆容器**：极端实时场景（回想第 2.1 节的 `no_std` 控制器）用 `heapless` 这类固定容量、栈上分配的容器，从根上杜绝运行时分配。

```rust,ignore
// ❌ 每帧都分配新 Vec —— 引入分配抖动
fn tick_bad(cloud: &PointCloud) -> Vec<Point> {
    let mut filtered = Vec::new();  // 每次都问操作系统要内存
    for p in &cloud.points {
        if p.z > 0.1 { filtered.push(*p); }
    }
    filtered
}

// ✅ 复用预分配好的缓冲区 —— 稳定、可预测
fn tick_good(cloud: &PointCloud, buf: &mut Vec<Point>) {
    buf.clear();  // 不释放内存，只是把长度归零，容量保留
    for p in &cloud.points {
        if p.z > 0.1 { buf.push(*p); }
    }
}
```

> **中高级视角**：注意 Rust 在这里给了你一个别的高级语言难有的优势——**分配是显式且可控的**。在 Java/Go/Python 里，对象随手就分配了，你甚至看不出来哪里在分配；而 Rust 里每次堆分配几乎都对应一个你写得出来的动作（`Vec::new`、`.clone()`、`Box::new`、`.to_string()`）。这意味着你能**审计**你的实时热路径、确保它零分配——甚至有工具能在测试里断言"这段代码没有发生任何堆分配"。这种对底层的精确控制，加上内存安全，正是 Rust 相比 C++（能控制但不安全）和 GC 语言（安全但不可控）的独特站位。第九部分讲实时性和部署时我们会再深入。

## 小结

- Rust 的**无畏并发**：所有权和借用规则延伸到线程间，**数据竞争在编译期被拒绝**（想跨线程共享就得用线程安全的 `Arc`，用 `Rc` 直接编不过）。
- **`Arc<T>`** 让多线程共享只读数据（clone 只加引用计数，不拷底层数据）；**`Arc<Mutex<T>>`** / **`Arc<RwLock<T>>`** 共享可变数据，锁把数据包在里面、离开作用域自动解锁（不会忘 unlock），读多写少用 `RwLock`。
- **channel** 是更好的并发模型——"通过通信共享内存"，发送即转移所有权，天然无竞争；`crossbeam` 提供 mpmc 和**有界 channel**（实现背压，实时系统关键）。感知/规划/控制就是用 channel 串起的并发节点，各按自己频率跑、用最新可用数据（控制节点 `try_recv`）。
- **线程 vs async**：CPU 密集（点云、推理、搜索）用**线程**（配 rayon）；IO 密集（网络、云、多路传感器）用 **async + tokio**。第一纪律：async 里不能有长时间不 `.await` 的计算。
- **实时性三坑**：锁竞争（临界区要短，进去拷一份就出来）、优先级反转（安全回路少和低优先级共享锁，偏爱 channel）、分配抖动（热路径预分配 + 复用缓冲区，极端场景用 `heapless`）。Rust 的分配显式可控，让你能审计出零分配的实时路径——这是它相对 C++ 和 GC 语言的独特站位。

本部分只剩最后一章了——**智驾常用 crate 生态速览**。学了这么多语言机制，下一章带你把它们和真实的库对上号：数学用什么、序列化用什么、推理用什么、中间件用什么，并给你一张能直接拿去做技术选型的建议表。
