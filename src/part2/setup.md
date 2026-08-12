# 2.1 搭建 Rust 开发环境

前面几章我们一直在纸上谈兵——画框图、聊行业、论证为什么用 Rust。从这一章开始，你要动手了。这一节的任务很实在：**把一套能写、能编、能查错的 Rust 环境装到你机器上**，理解每个工具是干嘛的，然后写出本书第一个和智驾沾边的程序——一个能打印障碍物清单的 "hello obstacle"。

别小看"装环境"。在很多团队里，一个新人卡在环境上耗掉两三天是常事：交叉编译不通、`rust-analyzer` 不跳转、`clippy` 报一堆看不懂的 lint。这一节我会把这些坑点连同背后的原理一起讲清，让你不只是"照着敲命令"，而是知道每一步在干什么。

## 一条命令装好一切：rustup

C++ 工程师第一次看到 Rust 的工具链管理会有点破防——不用手动折腾编译器版本、不用配环境变量地狱。一个叫 **rustup** 的工具（工具链管理器，toolchain manager）把这些全包了。

在 Linux / macOS 上：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

装完后，你的 `~/.cargo/bin` 会进 `PATH`。核实一下：

```bash
rustc --version   # 编译器本体
cargo --version   # 构建 & 包管理工具
rustup --version  # 工具链管理器
```

这里有三个角色，务必分清：

- **`rustc`**：真正的编译器。你几乎**从不直接调用它**，就像 C++ 工程师很少直接敲 `g++` 一长串参数一样。
- **`cargo`**：你天天打交道的工具。构建、跑测试、管依赖、生成文档、发布包——一个 `cargo` 全包了。它是 CMake + Make + Conan + Doxygen 的合体，而且不折磨人。
- **`rustup`**：管理"你装了哪些版本的 Rust、哪些目标平台、哪些组件"。

### stable、nightly 与 edition

Rust 每 6 周发一个 **stable（稳定版）**。还有 **nightly（每夜版）**，包含未稳定的实验特性。**智驾生产代码一律用 stable**——你要的是可预测和可复现，不是尝鲜。除非某个依赖强制要求 nightly（越来越少见），否则别碰。

```bash
rustup default stable      # 把默认工具链设为 stable
rustup update              # 升级到最新 stable
```

还有个容易和"编译器版本"搞混的概念：**edition（版次）**。Edition 是 Rust 的"语言方言年份"，目前有 2015 / 2018 / 2021 / 2024。它**不是**编译器版本——同一个新版 `rustc` 能编译所有 edition 的代码。Edition 的意义在于：Rust 想引入一些会破坏旧代码的语法改动（比如某个关键字），又不想把老项目全搞崩，于是用 edition 隔离。你在 `Cargo.toml` 里声明用哪个 edition，编译器就按那套规则解释你的代码。

**新项目一律用最新的 edition 2024**（本书写作时的最新稳定 edition）。它带来更顺手的语法（比如更符合直觉的闭包捕获、`gen` 块等）。

> **面试题**：edition 和 rustc 版本是一回事吗？
> **答**：不是。rustc 版本决定"你用的编译器多新、支持哪些特性"；edition 决定"你的代码按哪一年的语言规则来解析"。一个 2024 版的 rustc 可以同时编译 edition 2015 和 edition 2024 的 crate，让它们在一个项目里共存——这是 Rust 保证生态不断裂的关键设计。

## 三件套：rust-analyzer、clippy、rustfmt

装完 rustup 你就能编译了，但没有下面三样，你的开发体验会很痛苦。

### rust-analyzer：你的编辑器大脑

**rust-analyzer** 是 Rust 官方的语言服务器（LSP，Language Server Protocol），提供跳转定义、自动补全、类型提示、即时报错。在 VS Code 里装同名扩展即可；其它编辑器（Neovim、Emacs、CLion）也都支持 LSP。

一个 Rust 老手的日常里，rust-analyzer 的分量怎么强调都不过分——因为 Rust 的类型推导很强，你写代码时经常不写类型，全靠 rust-analyzer 把推导出来的类型贴在你眼前（inlay hints）。没有它，你就是在盲写。

### clippy：比编译器更啰嗦的老师

**clippy** 是官方 lint 工具，它在编译器之外额外检查你的代码风格和常见错误——比如"你这里可以用迭代器而不是手写循环""这个 `.clone()` 是多余的""这个浮点数比较很危险"。

```bash
rustup component add clippy
cargo clippy            # 检查当前项目
cargo clippy -- -D warnings   # 把所有 warning 当错误，CI 里常这么用
```

**在智驾项目里，clippy 几乎总是被强制开启并接入 CI**。它能揪出很多"能编过但不对劲"的代码。刚开始你可能觉得它烦，但它揪出的问题里有相当一部分是真 bug。习惯它。

### rustfmt：终结代码风格争论

**rustfmt** 自动格式化代码。团队里再也不用为"大括号该不该换行"吵架了——全交给它。

```bash
rustup component add rustfmt
cargo fmt               # 格式化整个项目
cargo fmt --check       # 只检查不改，CI 里用
```

> **真实项目里**：几乎每个正经的 Rust 仓库根目录都有一个 CI 流水线，最低配置是 `cargo fmt --check && cargo clippy -- -D warnings && cargo test`。这三行是 Rust 团队的"卫生底线"。你新加入一个团队，第一件事就是看它的 CI 配置，那里写着这个团队的工程标准。

## 一个 cargo 项目长什么样

来建第一个项目：

```bash
cargo new hello_obstacle
cd hello_obstacle
```

生成的目录结构：

```text
hello_obstacle/
├── Cargo.toml        # 项目清单：名字、版本、edition、依赖
├── Cargo.lock        # 锁定的依赖精确版本（自动生成，别手改）
└── src/
    └── main.rs       # 程序入口
```

`Cargo.toml`（读作 "cargo dot toml"）是项目的身份证：

```toml
[package]
name = "hello_obstacle"
version = "0.1.0"
edition = "2024"

[dependencies]
```

几个关键概念：

- **crate（箱）**：Rust 的编译单元和分发单元。一个 crate 要么是**二进制**（有 `main.rs`，能跑），要么是**库**（有 `lib.rs`，给别人用）。
- **package（包）**：一个 `Cargo.toml` 管理的东西，可以包含一个库 crate 加多个二进制 crate。
- **`Cargo.lock`**：记录你实际用的每个依赖的**精确版本和哈希**。库不提交它，应用（二进制）**必须提交**它——这样团队里每个人、CI、生产环境编出来的东西完全一致。这是可复现构建的基石，对需要长期维护和审计的车载软件极其重要。

常用命令：

```bash
cargo build            # debug 构建，产物在 target/debug/
cargo build --release  # release 构建，开优化，产物在 target/release/
cargo run              # 构建并运行
cargo test             # 跑测试
cargo check            # 只做类型检查，不生成可执行文件——最快，写代码时高频用
cargo doc --open       # 生成并打开文档
```

> **中高级视角**：`cargo check` 和 `cargo build` 的区别值得记住。`check` 跳过代码生成（codegen）和链接，只做前端分析，因此快得多。你写代码的循环应该是"改代码 → `cargo check`（或让 rust-analyzer 后台帮你 check）→ 反复"，只在真要运行时才 `build`。**Debug 构建和 Release 构建的性能能差 10~100 倍**——涉及性能的数字（比如点云处理耗时）永远要在 `--release` 下测，拿 debug 的数字下结论会闹笑话。

## 如何加依赖

Rust 的第三方库都在 [crates.io](https://crates.io) 上。加依赖有两种方式。手写 `Cargo.toml`：

```toml
[dependencies]
serde = { version = "1", features = ["derive"] }
serde_json = "1"
```

或者用命令（更推荐，能自动填最新版本）：

```bash
cargo add serde --features derive
cargo add serde_json
```

**版本号规则（semver，语义化版本）**：`"1"` 等价于 `"^1.0.0"`，意思是"允许任何 1.x.y，只要不到 2.0"。Rust 生态严格遵守 semver：主版本号不变就保证 API 兼容。这让"升级小版本拿 bug 修复"变得安全。

**features（特性）** 是 Rust 的条件编译开关。像 `serde` 的 `derive` feature，开启后才能用 `#[derive(Serialize)]` 这种派生宏。不开的话默认不编译那部分代码——这是 Rust 控制编译产物大小和依赖的机制，在资源受限的车载环境里很有用（你能精确裁剪掉不需要的功能）。

## 交叉编译到车载 ARM

你的开发机大概率是 x86-64（Intel/AMD）。但车载计算平台——英伟达 Orin、地平线征程、高通 Ride 等——用的是 **ARM64（aarch64）** 架构。你在 x86 上编出来的二进制**不能**直接在 ARM 上跑。你需要**交叉编译（cross-compilation）**：在 x86 机器上，编出能在 aarch64 上跑的程序。

Rust 对交叉编译的支持是同类语言里最舒服的之一。核心概念是 **target triple（目标三元组）**，形如 `aarch64-unknown-linux-gnu`，读作"架构-厂商-操作系统-ABI"。

第一步，装目标平台的标准库：

```bash
rustup target add aarch64-unknown-linux-gnu
```

第二步，你还需要一个能生成 ARM 机器码的**链接器（linker）**。Rust 自己能生成 ARM 目标代码，但把目标文件链接成最终可执行文件仍要一个交叉链接器（通常来自 GCC 交叉工具链）。装好后在 `.cargo/config.toml` 里告诉 cargo 用哪个链接器：

```toml
# .cargo/config.toml
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
```

然后：

```bash
cargo build --release --target aarch64-unknown-linux-gnu
```

产物在 `target/aarch64-unknown-linux-gnu/release/` 下，拷到车机上就能跑。

> **真实项目里**：手动配交叉工具链很容易踩坑（glibc 版本不匹配、缺 sysroot、依赖里有 C 库要一起交叉编译）。业界常用 [`cross`](https://github.com/cross-rs/cross) 这个工具——它把整套交叉编译环境封装进 Docker 容器，`cross build --target aarch64-unknown-linux-gnu` 一条命令搞定，绕开了本地环境的一堆麻烦。当你的项目开始依赖 C 库（比如包了某个厂商的 SDK）时，`cross` 能省下大量时间。

## no_std：当你连操作系统都没有

前面说的都默认你有个操作系统（Linux）。但智驾系统里有一类硬件——**MCU（微控制器）**，比如做底盘线控、安全监控的实时控制器——它可能：

- 没有操作系统，或者跑的是 RTOS（实时操作系统）；
- 内存只有几百 KB 到几 MB；
- 不能有不可预测的堆分配（动态内存申请可能失败、可能造成延迟抖动）。

这时你不能用 Rust 的**标准库（std）**，因为 std 假设了操作系统的存在（文件、线程、堆分配器、网络）。你要用 **`no_std`**——只依赖 Rust 的核心库 `core`（和可选的 `alloc`）。

在 crate 顶部声明：

```rust,ignore
#![no_std]
```

这一声明之后：

- `Vec`、`String`、`HashMap`、`println!` 这些**都没了**（它们在 std 里）。
- 你只有 `core` 提供的东西：基本类型、`Option`、`Result`、迭代器、切片操作、`Ordering`……语言的骨架还在。
- 如果硬件有堆，可以额外引入 `alloc` crate，拿回 `Vec`、`Box`、`String`。但很多硬实时代码会**刻意不用堆**，改用固定容量的容器（如 `heapless` crate 提供的 `heapless::Vec<T, N>`），把内存布局在编译期定死——这样就没有分配失败、没有分配抖动，满足硬实时要求。

```toml
# 嵌入式控制器常见依赖
[dependencies]
heapless = "0.8"     # 无堆分配的 Vec/String/队列，容量编译期固定
```

**为什么车载控制器爱用 no_std**：功能安全（ISO 26262）最高等级要求你能**论证**软件行为的确定性。一个会在运行时向操作系统申请内存、可能失败、耗时不定的操作，是"确定性"的敌人。`no_std` + 固定容量容器让整个程序的内存占用在编译期就可知、可证——这正是安全认证想要的。

> **中高级视角**：Rust 的一大杀手锏就是"同一门语言从内核态到应用层通吃"。感知/规划这些跑在 Orin 的 Linux 上、算力密集的模块用 std，尽情用 `Vec`、`tokio`、`rayon`；而底盘控制器上那段硬实时代码用 `no_std`。两边都是 Rust，工程师技能可迁移、代码可共享（把纯算法逻辑写成既能 std 又能 no_std 的 crate 是常见做法）。C++ 也能做到，但 Rust 的所有权模型让"无堆、无 GC、还内存安全"这件事第一次变得优雅。本书正文聚焦 std 环境（感知/规划/控制的主战场），no_std 你先建立概念，等真做嵌入式控制器时再深入。

## 你的第一个程序：hello obstacle

理论够了。我们复用第 1.3 节定义的 `Obstacle` 和 `ObstacleClass`，写一个能跑的小程序：造几个障碍物，筛出需要警惕的，打印出来。

把下面代码放进 `src/main.rs`：

```rust
/// 障碍物类别（来自第 1.3 节）
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObstacleClass {
    Vehicle,
    Pedestrian,
    Cyclist,
    Unknown,
}

/// 一个被感知到的障碍物（来自第 1.3 节）
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub id: u32,
    pub class: ObstacleClass,
    pub position: [f64; 3], // 自车坐标系 x,y,z (米)
    pub velocity: [f64; 3], // 速度矢量 (米/秒)
    pub size: [f64; 3],     // 长宽高 (米)
    pub heading: f64,       // 朝向角 (弧度)
    pub confidence: f32,    // 置信度 0~1
}

impl Obstacle {
    /// 到自车的水平距离（忽略 z）
    fn distance(&self) -> f64 {
        let [x, y, _] = self.position;
        (x * x + y * y).sqrt()
    }

    /// 是否值得警惕：在前方、够近、置信度够高
    fn is_threatening(&self) -> bool {
        self.position[0] > 0.0        // 在车前方
            && self.distance() < 50.0 // 50 米以内
            && self.confidence > 0.5  // 别被低置信度误报吓到
    }
}

fn main() {
    // 造一份障碍物清单（真实系统里这来自感知模块）
    let obstacles = vec![
        Obstacle {
            id: 42,
            class: ObstacleClass::Pedestrian,
            position: [30.0, -1.5, 0.0],
            velocity: [0.0, 0.3, 0.0],
            size: [0.5, 0.5, 1.7],
            heading: 1.57,
            confidence: 0.92,
        },
        Obstacle {
            id: 7,
            class: ObstacleClass::Vehicle,
            position: [80.0, 0.0, 0.0], // 太远
            velocity: [-5.0, 0.0, 0.0],
            size: [4.5, 1.9, 1.5],
            heading: 3.14,
            confidence: 0.99,
        },
        Obstacle {
            id: 99,
            class: ObstacleClass::Unknown,
            position: [-10.0, 2.0, 0.0], // 在车后方
            velocity: [0.0, 0.0, 0.0],
            size: [1.0, 1.0, 1.0],
            heading: 0.0,
            confidence: 0.3, // 低置信度
        },
    ];

    println!("感知到 {} 个障碍物，筛选需警惕目标：\n", obstacles.len());

    // 用迭代器筛选——这是地道的 Rust 风格，别写成手动 for 循环 + push
    let threats: Vec<&Obstacle> =
        obstacles.iter().filter(|o| o.is_threatening()).collect();

    if threats.is_empty() {
        println!("当前无需警惕的障碍物。");
    } else {
        for o in &threats {
            println!(
                "  #{:<3} {:?} 距离 {:.1} m  横向速度 {:+.2} m/s  置信度 {:.2}",
                o.id,
                o.class,
                o.distance(),
                o.velocity[1],
                o.confidence
            );
        }
    }
}
```

运行：

```bash
cargo run
```

输出：

```text
感知到 3 个障碍物，筛选需警惕目标：

  #42  Pedestrian 距离 30.1 m  横向速度 +0.30 m/s  置信度 0.92
```

只有 #42 通过了筛选：#7 太远（80 m），#99 在车后方且置信度太低。**这就是感知到规划之间那道"清单过滤"的最小雏形。**

几个值得注意的地道写法：

- 用 `iter().filter(...).collect()` 而不是手写循环——更清晰，clippy 也会推着你这么做。
- `filter` 拿到的是 `&Obstacle`（借用），我们没有拷贝任何障碍物数据。为什么这很重要？因为一份点云障碍物列表可能很大，**随手 clone 是性能杀手**——这正是第 2.2 节所有权的核心议题。
- `{:?}` 是 `Debug` 格式化，能用是因为我们给结构体派生了 `#[derive(Debug)]`。

试着故意搞点破坏：把 `filter(|o| o.is_threatening())` 里的 `o.is_threatening()` 改成 `o.is_threatning()`（拼错），`cargo check` 会立刻精确地告诉你哪里错了、甚至建议正确拼写。**这种"编译器帮你兜底"的体感，正是 Rust 开发的日常。** 下一章你会和这个编译器有更深入（有时是痛苦，但最终是幸福）的交往。

## 小结

- **rustup** 管工具链，**cargo** 是你天天用的构建/包管理工具，**rustc** 是你几乎不直接碰的编译器。生产代码一律用 **stable**，新项目用最新 **edition 2024**。
- 开发三件套：**rust-analyzer**（编辑器智能）、**clippy**（额外 lint，接 CI）、**rustfmt**（自动格式化）。CI 卫生底线是 `fmt --check && clippy -D warnings && test`。
- cargo 项目结构清晰，`Cargo.lock` 保证可复现构建（应用必须提交）；用 `cargo add` 加依赖，靠 **semver** 和 **features** 精确控制。
- **交叉编译**到车载 aarch64：`rustup target add` + 配交叉链接器，实战常用 `cross`。
- **no_std** 让 Rust 跑在无操作系统的嵌入式控制器上，配合固定容量容器满足硬实时和功能安全的确定性要求。
- 我们写出了 "hello obstacle"，第一次感受到用迭代器筛选障碍物、以及编译器兜底的体感。

下一章是本书第一个硬骨头也是最关键的一章：**所有权、借用与生命周期**。我们会用"管理一帧点云的内存"把这套让无数人又爱又恨的规则彻底讲透——它正是 Rust 敢承诺"没有内存错误"的底气所在。
