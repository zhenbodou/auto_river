# Rust 智能驾驶开发：从小白到工程师

一本以 **Rust 为主语言**、面向零基础读者、系统讲解**自动驾驶软件开发**的实战教程（mdBook 格式）。目标是建立中高级岗位所需的知识、工程方法与可复现项目证据；真实职级仍需要团队交付经验验证。

## 如何阅读

本书用 [mdbook](https://github.com/rust-lang/mdBook) 构建。

```bash
# 本地实时预览（自动打开浏览器，改动即时刷新）
mdbook serve --open

# 只构建静态站点，产物在 book/ 目录
mdbook build

# 构建后用浏览器打开
mdbook build && open book/index.html    # macOS
# xdg-open book/index.html               # Linux
```

如果没装 mdbook：

```bash
cargo install mdbook
```

## 内容结构

全书十个部分、46 节编号正文，另含前言、使用指南和后记：

| 部分 | 主题 | 核心 |
|---|---|---|
| 一 | 认识智能驾驶 | 分级、软件全景、五大模块、为什么用 Rust、行业地图 |
| 二 | 面向智驾的 Rust 基础 | 所有权、trait、错误处理、并发、性能、unsafe/FFI |
| 三 | 传感器、坐标系与数据 | 传感器、刚体变换、同步、数据建模、标定与健康监测 |
| 四 | 感知 | 图像、推理、点云、跟踪融合、数据闭环与量产评测 |
| 五 | 定位、建图与状态估计 | 卡尔曼滤波、组合导航、SLAM 与高精地图 |
| 六 | 预测与规划 | 行为预测、A*/Frenet/五次多项式、决策 |
| 七 | 控制 | 车辆模型、PID/Pure Pursuit/Stanley/LQR/MPC |
| 八 | 中间件与系统集成 | ROS 2/rclrs、Zenoh/DDS、消息、背压、可观测与资源预算 |
| 九 | 仿真、测试与安全 | CARLA/回放、测试、实时、功能安全、网络安全与安全论证 |
| 十 | 实战与职业 | 最小栈、逐章能力验收、量产化毕业项目、求职与术语表 |

## 特点

- **证据驱动**：核心算法兼顾推导、Rust 实现和失效分析；每节都有可交付验收，综合项目有延迟、鲁棒、安全和可复现硬门禁。
- **一条主线贯穿**：全书复用同一套数据结构（`Obstacle` / `Pose` / `TrajectoryPoint` / `ControlCommand`），第 1.3 节的"行人横穿减速让行"场景在第 10 部分被真正实现成一个可运行的最小自动驾驶栈。
- **务实**：坦诚讲 Rust 在智驾里的优势与边界，对照 C++/Python 的真实分工。

## 目录树

- `book.toml` — mdbook 配置（已启用搜索、MathJax 数学公式、代码折叠）
- `src/` — 全部章节 Markdown 源文件
- `src/SUMMARY.md` — 全书目录（增删章节改这里）
- `book/` — 构建产物（可删除，`mdbook build` 会重新生成）
