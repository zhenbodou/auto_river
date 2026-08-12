# 2.4 错误处理：Result、Option 与 anyhow

在服务器上，一个没处理好的错误顶多让请求失败、进程重启。在车上，错误处理是**功能安全**的一部分——传感器掉线、数据超时、标定文件读不出来，这些不是"异常情况"，而是**必然会发生的正常情况**，你的系统必须优雅地面对它们，而不是崩溃或者装作没看见。

这一章讲 Rust 怎么用类型系统把错误处理**变成强制的、显式的、编译器帮你盯着的**事情，以及一个真正区分初级和资深车载工程师的判断：**什么时候该 `panic`（让程序立刻崩），什么时候该优雅降级。** 这个判断错了，代价是安全事故。

## Rust 没有异常

先扭转一个观念。C++、Java、Python 用**异常（exception）**处理错误：出错就 `throw`，沿调用栈往上抛，直到某处 `catch`。这套机制的问题是——**错误路径是隐藏的**。你看一个函数签名 `Obstacle detect(const PointCloud&)`，完全看不出它会不会抛异常、抛什么。调用方很容易忘了 `catch`，然后运行时炸给你看。

Rust **根本没有异常**（`panic` 不是异常，后面讲）。它用两个普通的 `enum` 表达"可能没有值"和"可能出错"：

```rust
// 标准库里就长这样，没有任何魔法
enum Option<T> {
    Some(T),
    None,
}

enum Result<T, E> {
    Ok(T),
    Err(E),
}
```

区别是**哲学性的**：错误成了**返回值类型的一部分**，明明白白写在函数签名里。`fn detect(cloud: &PointCloud) -> Result<Vec<Obstacle>, DetectError>` 一眼就告诉你："这个函数可能失败，失败时给你一个 `DetectError`"。而且你**没法忽略它**——想拿到里面的 `Vec<Obstacle>`，就必须先处理 `Err` 那一支，编译器盯着你。

## Option：可能没有值

`Option<T>` 表达"要么有个 T，要么啥也没有"，取代了其它语言里那个万恶之源 null。第 2.3 节的 `Sensor::poll` 就返回 `Option<SensorMessage>`——没数据就是 `None`，天经地义，不是错误。

```rust,ignore
/// 从障碍物列表里找出指定 ID 的那个——可能根本不存在
fn find_obstacle(obstacles: &[Obstacle], id: u32) -> Option<&Obstacle> {
    obstacles.iter().find(|o| o.id == id)
}

fn main() {
    let obstacles = get_obstacles();
    match find_obstacle(&obstacles, 42) {
        Some(obs) => println!("找到了：距离 {:.1} m", obs.position[0]),
        None => println!("没有 ID=42 的障碍物"),
    }
}
```

**为什么这比 null 强？** 因为 `find_obstacle` 的返回类型是 `Option<&Obstacle>`，编译器**强制**你在用它之前处理 `None`。你不可能像在 C++/Java 里那样，直接对一个可能为 null 的指针解引用然后运行时空指针崩溃。**"十亿美元的错误"（null 的发明者 Tony Hoare 语）在 Rust 里从类型层面被堵死了。**

`Option` 有一堆好用的组合子，让你不必每次都写 `match`：

```rust,ignore
let obstacles = get_obstacles();

// 找到就取距离，找不到给默认值
let dist = find_obstacle(&obstacles, 42)
    .map(|o| o.position[0])      // Option<&Obstacle> -> Option<f64>
    .unwrap_or(f64::INFINITY);   // 没找到当作无穷远

// 找到且置信度够高才算数
let confident = find_obstacle(&obstacles, 42)
    .filter(|o| o.confidence > 0.8);
```

## Result 与 `?` 运算符

`Result<T, E>` 表达"要么成功给你 T，要么失败给你错误 E"。真实场景：读一个相机标定文件。

```rust,ignore
use std::fs;

fn load_calibration(path: &str) -> Result<Calibration, std::io::Error> {
    let content = fs::read_to_string(path)?;   // 读文件可能失败
    let calib = parse_calibration(&content)?;   // 解析也可能失败
    Ok(calib)
}
```

关键是那个 **`?` 运算符**。`fs::read_to_string(path)` 返回 `Result<String, io::Error>`。`?` 的语义是：

- 如果是 `Ok(content)`，把 `content` 取出来，继续往下走；
- 如果是 `Err(e)`，**立即从当前函数返回 `Err(e)`**。

所以 `?` 是"成功就解包、失败就提前返回错误"的语法糖。没有它，上面得写成一坨：

```rust,ignore
fn load_calibration_verbose(path: &str) -> Result<Calibration, std::io::Error> {
    let content = match fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => return Err(e),
    };
    let calib = match parse_calibration(&content) {
        Ok(c) => c,
        Err(e) => return Err(e),
    };
    Ok(calib)
}
```

对比一下就知道 `?` 有多重要——它让"正常逻辑"清晰地留在主线上，"出错就往上抛"退到一个字符里。**这是 Rust 错误处理体验的核心，你会天天用它。** 它保留了异常"自动向上传播"的便利，又没丢掉"错误在签名里显式可见、编译器强制处理"的安全性。

> **面试题**：`?` 能用在返回 `Result` 的函数里，也能用在返回 `Option` 的函数里，为什么？
> **答**：`?` 对 `Option` 同样成立——`Some(v)` 解包成 `v`，`None` 就提前 `return None`。它俩都是"有正常分支 + 短路分支"的 enum。`?` 还会在传播 `Result` 的 `Err` 时，自动调用 `From` trait 做错误类型转换——这正是下面 `thiserror` 能优雅工作的机制。

## 自定义错误 enum

`std::io::Error` 只能表达 IO 错误。但 `load_calibration` 可能因为多种原因失败：文件读不到、格式错、数值不合理。你需要一个能表达"本模块所有失败方式"的自定义错误类型——用 `enum`，一个变体一种失败：

```rust
#[derive(Debug)]
pub enum CalibError {
    Io(std::io::Error),         // 文件读不出来
    Parse(String),              // 格式错误
    OutOfRange { field: String, value: f64 }, // 数值超出物理合理范围
}
```

现在 `load_calibration` 可以返回 `Result<Calibration, CalibError>`，调用方 `match` 时能针对不同失败原因做不同处理——文件缺失可以用默认标定，但数值越界必须报警。

但手写这个 enum 有个烦人处：`?` 要能自动把 `io::Error` 转成 `CalibError::Io`，你得手动实现 `From<io::Error> for CalibError`；还要实现 `Display` 和 `std::error::Error` trait 才算个"合格的错误类型"。这些样板代码又臭又长。于是有了 `thiserror`。

## thiserror：给库写错误的利器

`thiserror` 是一个派生宏 crate，专门消灭上面那些样板。加依赖：

```toml
[dependencies]
thiserror = "1"
```

同样的错误类型，用 `thiserror` 写：

```rust,ignore
use thiserror::Error;

#[derive(Error, Debug)]
pub enum CalibError {
    #[error("读取标定文件失败")]
    Io(#[from] std::io::Error),   // #[from] 自动生成 From，让 ? 直接工作

    #[error("标定文件格式错误: {0}")]
    Parse(String),

    #[error("字段 {field} 的值 {value} 超出物理合理范围")]
    OutOfRange { field: String, value: f64 },
}
```

`#[derive(Error)]` 帮你实现了 `std::error::Error`；`#[error("...")]` 定义了每个变体的 `Display` 文案（可以插字段）；`#[from]` 自动生成 `From<io::Error>`，于是 `fs::read_to_string(path)?` 里的 `?` 能把 `io::Error` 无缝转成 `CalibError::Io`。样板全没了，功能一个不少。

```rust,ignore
fn load_calibration(path: &str) -> Result<Calibration, CalibError> {
    let content = std::fs::read_to_string(path)?;  // io::Error 自动转 CalibError::Io
    let calib = parse_calibration(&content)
        .map_err(|e| CalibError::Parse(e.to_string()))?;
    if calib.focal_length <= 0.0 {
        return Err(CalibError::OutOfRange {
            field: "focal_length".into(),
            value: calib.focal_length,
        });
    }
    Ok(calib)
}
```

## anyhow：给应用写错误的利器

`thiserror` 让你定义**精确的、结构化的**错误类型，好处是调用方能 `match` 每种错误分别处理。但有时你**不需要**这种精确——比如在 `main` 函数或某个顶层任务里，各种错误最终都是"打条日志、退出/重启"，你根本不打算区分它们。这时给每个模块定义精确错误 enum 就是过度设计。

这时用 `anyhow`。它提供一个"万能错误类型" `anyhow::Error`，能装下任何实现了标准 `Error` trait 的错误：

```toml
[dependencies]
anyhow = "1"
```

```rust,ignore
use anyhow::{Context, Result}; // 注意：anyhow::Result<T> = Result<T, anyhow::Error>

fn init_perception() -> Result<PerceptionPipeline> {
    let calib = load_calibration("camera.yaml")
        .context("初始化感知时加载相机标定失败")?;   // 加上下文信息
    let model = load_model("detector.onnx")
        .context("加载检测模型失败")?;
    let cloud = read_test_cloud("sample.pcd")
        .context("读测试点云失败")?;
    Ok(PerceptionPipeline::new(calib, model, cloud))
}
```

`anyhow` 的两个杀手锏：

1. **`?` 能吞下任意错误类型**——不管是 `CalibError`、`io::Error` 还是别的，都能自动转成 `anyhow::Error`，你不用为类型转换操心。
2. **`.context(...)`** 给错误加一层描述，最终打印时会形成一条**错误链**，告诉你"加载相机标定失败 → 原因是读文件失败 → 原因是文件不存在"。调试时这条链能直接把你带到出事地点，比一个光秃秃的 "No such file" 有用一万倍。

### 分工：库用 thiserror，应用用 anyhow

这是 Rust 生态的黄金约定，记牢：

- **库 crate（给别人调用的）用 `thiserror`**：暴露精确的、结构化的错误类型，让**调用方**有能力分门别类地处理。你写一个感知库，别人调你的 `detect()`，他需要知道"是模型没加载"还是"输入点云为空"来决定怎么办。你不能替他决定"反正都报错退出"。
- **应用 crate（最终跑起来的二进制）用 `anyhow`**：在顶层，各种错误殊途同归——记日志、告警、降级或退出。用 `anyhow` 省掉定义一堆错误类型的功夫，还白得错误链。

> **真实项目里**：你手边的 `inf/` 推理 SDK 就是这个约定的现实样本——它作为一个可复用的库，用 `thiserror` 定义自己的错误枚举，让上层调用者能精确区分"配置解析失败""模型加载失败""推理执行失败"；而最外层那个命令行工具（应用层）则可以用 `anyhow` 把这些错误统一收口、打上下文、汇报给用户。一个成熟 Rust 项目里，你几乎总能看到这两个 crate 同时出现，各司其职。

## panic：Rust 的"直接崩溃"

现在讲那个我们一直搁置的东西：**`panic`**。当程序遇到一个它认为"无法继续、继续下去只会更糟"的情况，它会 `panic`——打印错误信息和调用栈，然后**终止当前线程**（默认整个程序退出）。

触发 panic 的常见方式：

```rust,ignore
let obs = find_obstacle(&obstacles, 42).unwrap(); // None 时 panic！
let obs = find_obstacle(&obstacles, 42).expect("必须存在 ID=42"); // 带信息的 panic
let x = arr[100];       // 数组越界，panic（注意：不是像 C 那样读垃圾内存！）
let y = 1 / divisor;    // divisor 为 0 时 panic
panic!("到达了不该到达的代码分支"); // 手动引爆
```

**`unwrap()` 和 `expect()` 是 panic 的常见来源**——它们说"我打赌这里一定是 `Ok`/`Some`，赌输了就崩"。

关键区别，一定要分清：

- **`Result`/`Option` 是可恢复错误（recoverable）**：调用方有机会处理、降级、重试。传感器掉线、文件读不到，都属此类。
- **`panic` 是不可恢复错误（unrecoverable）**：代表一个**程序 bug 或违反了不该违反的前提**——数组越界、除零、断言失败。它的哲学是"与其带着已损坏的状态继续跑，不如立刻停下"。

## 车载系统的灵魂拷问：什么时候绝不能 panic

这是全章最重要、也最能体现工程判断力的部分。**"该不该 panic"没有放之四海的答案，取决于这段代码在系统里的角色。**

**绝对不能 panic 的地方：正在跑的、安全攸关的实时控制回路。** 想象控制模块在 100 Hz 地给底盘发指令，某一帧它 `unwrap()` 了一个 `None`——线程 panic，控制指令**断流**。车此刻正以 100 km/h 行驶，方向盘和油门突然没人管了。**这不是崩溃报错，这是事故。** 所以在感知/规划/控制的主循环里：

- **禁用 `unwrap()`/`expect()`**——每一个都是一颗定时炸弹。用 `match`、`unwrap_or`、`?` 优雅处理所有分支。
- 用 clippy 的 lint（`unwrap_used`、`expect_used`）在 CI 里把它们直接禁掉。
- 传感器给了一帧坏数据？**降级**——用上一帧的预测外推、进入安全模式减速、把控制权交给冗余系统，**就是不能让线程死掉**。

```rust,ignore
// ❌ 控制回路里的定时炸弹
fn control_tick(traj: &Trajectory) -> ControlCommand {
    let target = traj.first().unwrap(); // 轨迹为空就 panic → 控制断流
    compute_command(target)
}

// ✅ 优雅降级：拿不到有效轨迹就发安全指令
fn control_tick(traj: &Trajectory) -> ControlCommand {
    match traj.first() {
        Some(target) => compute_command(target),
        None => {
            log::error!("轨迹为空，进入安全刹停");
            ControlCommand { steering_angle: 0.0, throttle: 0.0, brake: 0.3 }
        }
    }
}
```

**那什么时候 fail-fast（快速失败、立刻 panic）反而是对的？** 在**启动/初始化阶段**，以及**检测到"绝不该发生"的内部逻辑矛盾**时。

- **启动时**：标定文件缺失、配置非法、模型加载不出来——这时候车还没动，与其带着残缺的配置**假装能开**（那才真的危险），不如立刻 panic、拒绝启动、让人来修。**一个配置错误的自动驾驶系统绝不该被允许上路。**
- **内部不变量被打破时**：如果代码里出现了"按设计根本不可能发生"的状态（比如障碍物 ID 在一个本应去重的表里出现两次），这说明有 bug，继续跑只会用错误状态污染更多计算、甚至误导决策。这时 fail-fast 让 bug 在测试阶段就暴露，好过它悄悄潜伏到量产车上。

一句话总结这个判断：**车还没动、或发现自己已经错乱 → fail-fast 是负责任的；车正在动、面对的是可预期的外部故障 → 绝不能 panic，必须降级。**

> **中高级视角**：真实的车载系统是**分层**的。单个模块进程可以采用"检测到自身逻辑错乱就 fail-fast 退出"的策略——因为它上面有一个**监控/看门狗（watchdog）**层，一旦发现某模块死了，会立刻拉起备份、切到降级模式、或触发最小风险机动（MRM，minimum risk maneuver，通常是安全靠边停车）。所以"这个进程可以 panic"和"整车必须永远安全"并不矛盾——安全是靠**系统级冗余**保证的，不是靠祈祷每个进程都不出错。理解这一分层，你才能在每个具体模块里做出正确的"panic 还是降级"决策。

## 用类型系统表达传感器掉线与超时

回到最实在的问题：传感器掉线、超时怎么用类型表达？别用"返回一个特殊的 -1"或者"塞个全 0 的帧"这种 C 风格的哨兵值（sentinel value）——那正是 bug 的温床。用类型把每种状态显式建模：

```rust,ignore
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SensorError {
    #[error("{name} 掉线（已 {elapsed_ms} ms 无数据）")]
    Timeout { name: String, elapsed_ms: u64 },

    #[error("{name} 数据损坏: {reason}")]
    Corrupt { name: String, reason: String },

    #[error("{name} 时间戳倒退，可能时钟异常")]
    TimestampRegression { name: String },
}

/// 带超时的读取：明确区分“正常有数据”“正常暂时没数据”“出故障了”
fn read_with_timeout(sensor: &mut dyn Sensor, budget_ms: u64)
    -> Result<Option<SensorMessage>, SensorError>
{
    // Ok(Some(msg)) : 拿到一帧
    // Ok(None)      : 这次轮询暂时没数据，但传感器健康（正常，不是错误）
    // Err(Timeout)  : 超过预算还没数据，判定掉线（真故障）
    // ...
    todo!()
}
```

看这个签名的表达力：**`Result<Option<SensorMessage>, SensorError>`** 把三种本质不同的情况彻底分开了——"拿到数据"、"健康但暂时没数据"、"出故障了"。调用方用 `match` 处理时，编译器逼着它把"传感器掉线"这一支考虑进去。你**没法**像用哨兵值那样"忘了检查"。上层收到 `Err(Timeout)` 就能触发对应的降级逻辑（切到冗余传感器、进入安全模式）。**把"传感器可能掉线"这个残酷现实编码进类型，编译器就成了帮你不漏处理任何故障分支的伙伴**——这正是 Rust 类型系统在安全攸关系统里最大的价值。

## 小结

- Rust **没有异常**，用 `Option<T>`（可能没值，取代 null）和 `Result<T, E>`（可能出错）这两个普通 enum 把错误变成**签名里显式可见、编译器强制处理**的东西。
- **`?` 运算符**是"成功解包、失败提前返回"的语法糖，让主逻辑清晰、错误自动上抛，还会顺手做错误类型转换。
- **`thiserror` 给库用**——定义精确、结构化的错误 enum，让调用方能分类处理；**`anyhow` 给应用用**——万能错误类型 + `.context()` 错误链，顶层统一收口。`inf/` SDK 正是这个分工的现实样本。
- **`panic` 是不可恢复错误**（bug/越界/断言失败），`unwrap`/`expect` 是它的常见引信。
- **车载系统的判断**：正在跑的安全攸关回路**绝不能 panic**，必须优雅降级（禁 unwrap，用 clippy 把它 lint 掉）；而**启动阶段配置/标定非法、或检测到内部逻辑错乱时，fail-fast 反而负责任**。整车安全靠系统级冗余（看门狗 + MRM），不是靠每个进程都不出错。
- 用 `Result<Option<T>, SensorError>` 这样的类型把"有数据/暂无数据/掉线"三态显式分开，让编译器帮你不漏处理任何故障分支。

下一章进入本部分第二个硬骨头——**并发与异步**：线程、`Arc`/`Mutex`、channel，把感知/规划/控制建模成用 channel 通信的并发节点，`async`/`tokio` 何时用，以及从实时性视角看锁竞争、优先级反转和分配抖动这些能要命的坑。
