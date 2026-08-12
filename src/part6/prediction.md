# 6.1 行为预测

第一部分那一帧数据里，感知告诉我们"前方 30 米有个行人，横向速度 0.3 m/s 朝车道"，然后我们轻描淡写地说了一句"预测模块推断他有 0.7 的概率横穿"。这一章就是要把那句轻描淡写拆开——**你怎么从一堆历史位置和一张地图，算出别人未来几秒会去哪，还带上概率？**

先给你一个心理定位：预测（prediction，行业里也叫 behavior prediction、motion prediction）夹在感知和规划中间，是整条流水线里**最反直觉**的一环。感知有"真值"可以对（框到底准不准，去人工标注一下就知道）；控制有物理定律兜底（车就是会按牛顿力学动）。而预测要回答的是"那个人心里在想什么"——**一个本质上不可观测的东西**。你永远拿不到真值，因为对方自己可能都还没决定。这决定了预测这行的一切工程哲学：**我们不追求猜对，我们追求不猜漏。**

## 预测到底在解什么问题

把它写成一个函数签名，你立刻就懂了：

```rust,ignore
/// 输入：一个障碍物的历史轨迹 + 周边地图上下文
/// 输出：若干条可能的未来轨迹，每条带一个概率
fn predict(
    history: &[TrajectoryPoint],   // 过去 1~2 秒的观测，比如 10~20 个点
    map: &HdMap,                    // 车道、路口、人行横道……
    neighbors: &[Obstacle],         // 周围其他交通参与者（博弈要用）
) -> Vec<PredictedTrajectory>;

pub struct PredictedTrajectory {
    pub points: Vec<TrajectoryPoint>,  // 复用 1.3 节的 TrajectoryPoint
    pub probability: f64,              // 该模态的概率，所有模态之和 = 1
}
```

注意这个签名里已经藏了三件大事：

1. **输入是历史 + 地图 + 邻居**。只看历史（纯运动学）能做一个粗糙的预测，但"这条路只能左转"这种信息在地图里，"前车急刹所以我也要刹"这种信息在邻居里。地图和交互是把预测从"外推"升级到"理解"的关键。
2. **输出是一个 `Vec`，不是一条**。这就是 1.3 节反复强调的**多模态（multi-modal）**。一个路口前的车，可能直行、可能左转、可能右转，这是三条**质的不同**的未来，你不能把它们平均成"往前偏左一点"——那条平均轨迹哪儿都不是，撞谁都来不及。
3. **每条带 `probability`**。规划要用这个概率做期望风险的权衡：0.7 的横穿和 0.01 的横穿，处理方式完全不同。

预测的时间跨度（horizon）通常是 **3~8 秒**。太短了规划来不及反应，太长了纯属算命——3 秒后的世界早被无数次交互改写了。

## 为什么预测这么难

新人常觉得"不就是外推嘛，v 乘 t 加上去"。真到项目里你会被现实反复教育，难点有三层：

**第一，意图不可观测。** 同样是"车在车道里以 10 m/s 直行"，他下一秒是保持、是变道超车、还是要减速右转进匝道？这三者在**当前这一帧**里可能长得一模一样。你手上只有位置和速度，意图藏在驾驶员脑子里。预测的本质是**从可观测的运动，反推不可观测的意图，再从意图正推未来的运动**。

**第二，博弈与交互。** 交通不是每个人独立运动，而是一群人**互相看着对方做决定**。你要并线，旁边车看到你打灯，他可能让、也可能加速堵你。这意味着"障碍物的未来"依赖"你的未来"，而"你的未来"（规划的输出）又依赖"障碍物的未来"（预测的输出）——**鸡生蛋，蛋生鸡**。后面讲"交互式预测"时我们会正面刚这个循环。

**第三，长尾与稀有性。** 99% 的时间大家都规规矩矩，但吃掉你大部分工程精力的是那 1%：突然横穿的行人、逆行的电动车、鬼探头。这些样本在数据里极少，学习模型天然学不好，却恰恰是安全最在意的。

> **中高级视角**：初级工程师问"我的预测准确率（ADE/FDE）多少"，高级工程师问"我的预测在什么场景下会漏掉高危模态，规划有没有兜底"。指标（后面讲）是拿来发论文和横向对比的；**漏报（missed mode）才是会出事故的**。一个平均误差很小但偶尔漏掉横穿行人的预测器，比一个平均误差大一点但从不漏报的预测器危险得多。

## 方法演进：从物理外推到理解意图

预测方法这些年基本沿着一条"越来越懂上下文"的路径演进。我们一层层往上爬，每层都给你数学和代码。

### 第 0 层：物理模型（constant velocity / CTRV）

最朴素的假设：**对方会保持当前的运动状态**。这类模型不看地图、不看意图，纯运动学外推。别看它简单，工业界至今在用——因为它**永远不会崩**（没有神经网络会给你一个飞到天上的预测），而且对**短时间跨度**（1 秒内）出奇地准。它是你所有花哨模型的**保底基线（fallback baseline）**。

**恒速模型（Constant Velocity, CV）** 假设速度矢量不变：

$$
\begin{aligned}
x(t) &= x_0 + v_x \cdot t \\
y(t) &= y_0 + v_y \cdot t
\end{aligned}
$$

就这么简单。位置等于初始位置加速度乘时间。它的缺陷也一眼可见：**车不会永远直线走**，一进弯道它就飞出去了。

**恒定转弯率与速度模型（Constant Turn Rate and Velocity, CTRV）** 补上了这个缺陷。它假设车以恒定的速度 `v` 和恒定的偏航角速度（yaw rate）`ω` 运动——也就是走一段圆弧。状态向量取 `[x, y, θ, v, ω]`（位置、朝向、速率、转弯率）。它的运动方程分两种情况：

当 `ω ≈ 0`（几乎直行）时退化成恒速：

$$
\begin{aligned}
x(t) &= x_0 + v \cos\theta_0 \cdot t \\
y(t) &= y_0 + v \sin\theta_0 \cdot t \\
\theta(t) &= \theta_0
\end{aligned}
$$

当 `ω ≠ 0`（转弯）时走圆弧，积分 `θ(t) = θ_0 + ω t` 得到：

$$
\begin{aligned}
x(t) &= x_0 + \frac{v}{\omega}\big(\sin(\theta_0 + \omega t) - \sin\theta_0\big) \\
y(t) &= y_0 + \frac{v}{\omega}\big(-\cos(\theta_0 + \omega t) + \cos\theta_0\big) \\
\theta(t) &= \theta_0 + \omega t
\end{aligned}
$$

这个 `v/ω` 就是圆弧的半径。别死记公式，理解它：**车以角速度 ω 绕着一个半径 R = v/ω 的圆心转**，把圆周运动的位置写出来就是上面那两行。

那 `ω` 从哪来？从历史轨迹里**估**出来——用最近两帧的朝向差除以时间差，`ω ≈ (θ_1 - θ_0) / Δt`。这也是为什么预测的输入要带"历史"而不只是"当前"：你得从历史里把这些运动学参数估出来。

下面是**可运行**的 Rust 实现。CV 和 CTRV 都给，注意 CTRV 里对 `ω→0` 的数值处理——除以一个接近零的数是经典的数值陷阱。

```rust
// Cargo.toml 无需额外依赖，标准库即可

use std::f64::consts::PI;

/// 复用 1.3 节的 TrajectoryPoint
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub v: f64,
    pub heading: f64,
}

/// CTRV 状态：位置、朝向、速率、转弯率
#[derive(Debug, Clone, Copy)]
pub struct CtrvState {
    pub x: f64,
    pub y: f64,
    pub theta: f64, // 朝向角 (rad)
    pub v: f64,     // 速率 (m/s)
    pub omega: f64, // 偏航角速度 (rad/s)
}

impl CtrvState {
    /// 从至少两个历史点估计出 CTRV 状态。
    /// 真实项目里这一步往往用 EKF/UKF 平滑，这里用差分做直观演示。
    pub fn from_history(history: &[TrajectoryPoint]) -> Option<Self> {
        let n = history.len();
        if n < 2 {
            return None;
        }
        let last = history[n - 1];
        let prev = history[n - 2];
        let dt = (last.t - prev.t).max(1e-3); // 防止除零

        // 速率：优先用感知给的 v，退化时用位置差分
        let v = if last.v > 1e-3 {
            last.v
        } else {
            let dx = last.x - prev.x;
            let dy = last.y - prev.y;
            (dx * dx + dy * dy).sqrt() / dt
        };

        // 转弯率：相邻朝向差，注意角度归一化到 (-pi, pi]
        let dtheta = normalize_angle(last.heading - prev.heading);
        let omega = dtheta / dt;

        Some(CtrvState {
            x: last.x,
            y: last.y,
            theta: last.heading,
            v,
            omega,
        })
    }

    /// 向前预测 dt 秒后的状态（单步递推）
    pub fn propagate(&self, dt: f64) -> CtrvState {
        // 关键数值处理：omega 接近 0 时用恒速直线公式，
        // 否则 v/omega 会爆炸成 NaN/Inf。这个阈值是工程经验值。
        const EPS: f64 = 1e-4;
        let (nx, ny, ntheta) = if self.omega.abs() < EPS {
            (
                self.x + self.v * self.theta.cos() * dt,
                self.y + self.v * self.theta.sin() * dt,
                self.theta,
            )
        } else {
            let r = self.v / self.omega; // 转弯半径
            let th_new = self.theta + self.omega * dt;
            (
                self.x + r * (th_new.sin() - self.theta.sin()),
                self.y + r * (-th_new.cos() + self.theta.cos()),
                normalize_angle(th_new),
            )
        };
        CtrvState {
            x: nx,
            y: ny,
            theta: ntheta,
            v: self.v,
            omega: self.omega,
        }
    }

    /// 生成一条 horizon 秒、步长 step 秒的预测轨迹
    pub fn predict(&self, horizon: f64, step: f64) -> Vec<TrajectoryPoint> {
        let mut out = Vec::new();
        let mut state = *self;
        let mut t = 0.0;
        while t < horizon - 1e-9 {
            state = state.propagate(step);
            t += step;
            out.push(TrajectoryPoint {
                t,
                x: state.x,
                y: state.y,
                v: state.v,
                heading: state.theta,
            });
        }
        out
    }
}

/// 把任意角度归一化到 (-pi, pi]，预测/控制里到处都要用它
fn normalize_angle(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a <= -PI {
        a += 2.0 * PI;
    }
    a
}

fn main() {
    // 造一段"正在向左微转"的历史
    let history = vec![
        TrajectoryPoint { t: 0.0, x: 0.0, y: 0.0, v: 10.0, heading: 0.00 },
        TrajectoryPoint { t: 0.1, x: 1.0, y: 0.0, v: 10.0, heading: 0.02 },
    ];
    let state = CtrvState::from_history(&history).unwrap();
    let traj = state.predict(3.0, 0.5); // 预测 3 秒，每 0.5 秒一个点
    for p in &traj {
        println!("t={:.1}s  x={:6.2}  y={:6.2}  heading={:.3}", p.t, p.x, p.y, p.heading);
    }
}
```

跑一下你会看到轨迹是一条微微左弯的弧线——因为我们从历史里估出了一个小小的正 `omega`。把 `heading` 差分调大，弯就更明显。**这就是一个能上车的保底预测器**，几微秒出结果，永远不发散。

> **真实项目里**：CV/CTRV 常常不是拿来做主预测，而是有两个用途。一是**兜底**：当学习模型输出可疑（比如轨迹飞出道路）或超时，立刻回退到物理外推。二是**校验**：拿物理预测和学习预测对比，差太多就降低置信度。永远给你的花哨模型留一条物理退路，这是功能安全的基本素养。

### 第 1 层：基于地图的车道跟随预测

物理模型有个致命盲区：**它不知道路长什么样**。一辆车在弯道里，CTRV 假设它保持当前转弯率，但真实驾驶员是**跟着车道走**的——车道拐多少他拐多少。于是有了**基于地图（map-based）** 的预测：不外推运动，而是**把障碍物"吸附"到最近的车道中心线，然后假设它沿车道中心线以当前速度前进**。

思路三步走：

1. **关联车道**：找到障碍物当前所在（或最可能所在）的车道。
2. **投影**：把它投影到车道中心线上，得到一个纵向位置 `s`。
3. **沿线外推**：以当前速率沿中心线把 `s` 往前推，再把 `(s, 0)` 转回笛卡尔坐标。

这里已经用到了**中心线的弧长参数化**——沿着车道中心线走的距离 `s`。这个 `s` 就是下一章 Frenet 坐标的灵魂，我们提前打个照面。代码骨架：

```rust,ignore
/// 车道中心线：一串有序的采样点，我们预计算每个点的累积弧长 s
pub struct Centerline {
    pub pts: Vec<(f64, f64)>, // 世界坐标下的中心线点
    pub s: Vec<f64>,          // 每个点对应的累积弧长，s[0] = 0
}

impl Centerline {
    pub fn new(pts: Vec<(f64, f64)>) -> Self {
        let mut s = vec![0.0];
        for i in 1..pts.len() {
            let (x0, y0) = pts[i - 1];
            let (x1, y1) = pts[i];
            let d = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
            s.push(s[i - 1] + d);
        }
        Centerline { pts, s }
    }

    /// 把世界坐标点投影到中心线，返回最近点的弧长 s（简化：找最近采样点）
    pub fn project(&self, x: f64, y: f64) -> f64 {
        let mut best = (f64::INFINITY, 0.0);
        for i in 0..self.pts.len() {
            let (px, py) = self.pts[i];
            let d2 = (px - x).powi(2) + (py - y).powi(2);
            if d2 < best.0 {
                best = (d2, self.s[i]);
            }
        }
        best.1
    }

    /// 给定弧长 s，插值出世界坐标（线性插值，够用）
    pub fn point_at(&self, s_query: f64) -> (f64, f64) {
        // 边界处理
        if s_query <= 0.0 {
            return self.pts[0];
        }
        if s_query >= *self.s.last().unwrap() {
            return *self.pts.last().unwrap();
        }
        // 找到 s_query 落在哪一段，线性插值
        for i in 1..self.s.len() {
            if self.s[i] >= s_query {
                let ratio = (s_query - self.s[i - 1]) / (self.s[i] - self.s[i - 1]);
                let (x0, y0) = self.pts[i - 1];
                let (x1, y1) = self.pts[i];
                return (x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0));
            }
        }
        *self.pts.last().unwrap()
    }
}

/// 基于地图的车道跟随预测：沿中心线以当前速度前进
pub fn predict_lane_follow(
    obs: &Obstacle,
    lane: &Centerline,
    horizon: f64,
    step: f64,
) -> Vec<TrajectoryPoint> {
    let speed = (obs.velocity[0].powi(2) + obs.velocity[1].powi(2)).sqrt();
    let s0 = lane.project(obs.position[0], obs.position[1]);
    let mut out = Vec::new();
    let mut t = step;
    while t <= horizon + 1e-9 {
        let s = s0 + speed * t; // 沿弧长前进
        let (x, y) = lane.point_at(s);
        out.push(TrajectoryPoint { t, x, y, v: speed, heading: 0.0 });
        t += step;
    }
    out
}
```

`Obstacle` 沿用 1.3 节的定义（`position`、`velocity` 等字段）。这个预测器立刻解决了弯道问题——**因为它跟着路走，不管路怎么拐**。代价是它需要高精地图，且在**路口这种一对多**（一条进入车道连着直行/左转/右转多条出口车道）的地方，你得为每条候选车道各生成一条预测——**这天然就产生了多模态**。

### 第 2 层：机动意图分类

车道跟随假设"对方会老实待在车道里"，但变道、超车、进匝道怎么办？这就需要显式地**分类机动意图（maneuver intent）**：把连续的驾驶行为切成几个离散类别——`{ 车道保持, 左变道, 右变道, 左转, 右转, 减速停车 }`——先分类"他想干嘛"，再对每个意图生成对应轨迹。

早期用手工特征 + 分类器（比如看横向速度、离车道线距离、转向灯状态），现在多用小网络。但**接口设计**比模型本身更值得你记住：

```rust,ignore
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Maneuver {
    LaneKeep,
    LaneChangeLeft,
    LaneChangeRight,
    TurnLeft,
    TurnRight,
    Stop,
}

/// 意图分类的输出：每个机动一个概率，和为 1
pub struct IntentDistribution {
    pub probs: Vec<(Maneuver, f64)>,
}

/// 完整的"意图 → 轨迹"两段式预测
pub fn predict_intent_based(
    obs: &Obstacle,
    intents: &IntentDistribution,
    map: &HdMap,
) -> Vec<PredictedTrajectory> {
    intents
        .probs
        .iter()
        .filter(|(_, p)| *p > 0.05) // 剪掉可以忽略的低概率模态
        .map(|(m, p)| {
            let points = generate_trajectory_for(*m, obs, map); // 每种意图各自的生成逻辑
            PredictedTrajectory { points, probability: *p }
        })
        .collect()
}
```

看到那个 `> 0.05` 的过滤没？这是工程里的**模态剪枝**——你不可能把 200 条低概率轨迹都塞给规划，规划会算爆。但**阈值定多少是个良心活**：定高了漏掉罕见但危险的模态（行人突然横穿概率可能只有 0.03，但你敢剪吗？），定低了规划过度保守寸步难行。这里没有标准答案，只有场景相关的权衡——横穿行人绝不能按纯概率剪，得让安全逻辑一票否决。

### 第 3 层：学习方法（LSTM / Transformer / VectorNet）

上面三层都有个共同弱点：**手工设计**。特征是人挑的、意图类别是人定的、轨迹生成规则是人写的。真实交通的复杂度早晚超出手工建模能力，于是进入**数据驱动**时代。这里只讲概念和直觉，实现细节属于第四部分的推理框架和专门的深度学习课程。

- **LSTM/RNN 时序编码**：把历史轨迹当成一个序列喂进循环网络，让它自己学"这种运动模式接下来通常会怎样"。解决了"手工挑特征"的问题，但对**地图和交互**的建模很弱。

- **Transformer 与注意力**：用自注意力（self-attention）让每个交通参与者"关注"到跟它相关的其他参与者和地图元素。这天然适合建模**交互**——注意力权重就是"谁在乎谁"。现在主流预测模型（如各家的 motion transformer）几乎都以它为骨架。

- **VectorNet / 图表示**：一个关键洞见——**把地图和轨迹都表示成"矢量"（一段段折线 polyline），而不是渲染成图片再用 CNN**。车道线是折线、历史轨迹是折线、人行横道是折线，把它们统一编码成图的节点，用图神经网络（GNN）聚合。这比"把地图画成鸟瞰图喂 CNN"高效得多，也是 VectorNet 及其后继（如 LaneGCN）的核心思想。

- **多模态输出的表达**：学习模型怎么吐出"多条带概率的轨迹"？主流两招。**基于锚点（anchor-based）**：预定义一批典型轨迹模板（anchors），网络对每个模板输出一个概率 + 一个微调偏移——这和目标检测里的 anchor 一个套路。**基于意图/目标点（goal-based）**：先预测"他想去哪个终点（goal）"的分布，再对每个 goal 回归一条轨迹。无论哪种，损失函数都要用 **"胜者为王"（winner-takes-all）** 的变体——只惩罚离真值最近的那条模态，否则网络会把所有模态学成一样的平均轨迹，多模态就塌了（mode collapse）。

> **面试题**：为什么预测网络直接回归多条轨迹容易"模态塌缩"，怎么解决？
> **答**：如果对每条输出都用真值算 L2 损失，网络发现"输出所有模态的平均"能让总损失最小，于是所有模态收敛成同一条模糊的平均轨迹，多模态名存实亡。解决办法是 winner-takes-all：每个样本只有真值，只对**最接近真值的那条**模态回传梯度（外加一个分类损失学各模态的概率），逼着不同模态去覆盖不同的行为。

## 多模态输出到底怎么表达

无论用哪层方法，最终交给规划的数据结构就是本章开头那个 `Vec<PredictedTrajectory>`。但有几个工程细节决定它好不好用：

- **概率归一化**：所有模态概率之和必须为 1，规划才能算期望。剪枝之后记得重新归一化。
- **协方差 / 不确定性**：高级系统里每个轨迹点不只是 `(x, y)`，还带一个位置协方差——**越远的点越不确定**，椭圆越大。规划做碰撞检测时用的是"概率占据"，不是一个点。
- **一致性**：同一个障碍物在连续帧之间的预测不能乱跳（这一帧说他要左转，下一帧说右转，规划会被晃晕）。工程上要做**时序平滑**。

```rust
/// 规划真正想要的、带不确定性的预测点
pub struct PredictedPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub cov_xx: f64, // 位置协方差，随 t 增大而增大
    pub cov_yy: f64,
    pub cov_xy: f64,
}
```

## 工程现实：预测与规划的耦合，与交互式预测

现在回到前面挖的那个"鸡生蛋"的坑。**开环预测**（open-loop）假设障碍物的未来与自车无关——先预测所有人，再规划自己。这在大部分场景够用，但在**强交互**场景（并线、无保护左转、路口博弈）会失效：你预测"旁边车会一直直行"，于是决定并进去，可对方明明是看到你要并才会让/会堵。你的预测没考虑"他在看你"。

于是有了**交互式预测（interactive prediction）** 和**闭环（closed-loop）** 的思路，几种典型做法：

- **条件预测（conditional prediction）**：预测时把"自车的候选计划"作为条件输入——"如果我并线，他大概率会让；如果我不动，他就直行"。规划遍历自己的几个候选动作，对每个动作问一次预测，选综合最优的。这本质是把博弈显式化。
- **联合预测（joint prediction）**：不再逐个独立预测每个障碍物，而是**一次性预测所有参与者的联合未来**，让模型内部自己处理交互一致性（A 让了 B，B 就能过）。独立预测会产生自相矛盾的组合（两辆车都以为对方会让，预测出双双直行然后"相撞"）。
- **博弈论 / POMDP 建模**：把整个场景建成一个多智能体博弈或部分可观测马尔可夫决策过程（POMDP），意图是隐状态。这在数学上最优美，但计算量大，实时性是硬伤。我们在 6.3 决策那章会再深入这块。

> **中高级视角**：预测和规划到底该不该分开，是这几年架构层面的大争论。传统栈是"预测→规划"两个独立模块（好处：可解释、可分别测试、可兜底）。而端到端（end-to-end）和一体化（如各家的 "joint perception-prediction-planning"）主张把它们揉进一个网络联合优化（好处：不丢信息、能建模交互；坏处：黑盒、难验证、出了事故说不清)。目前量产车上**主流仍是模块化 + 局部交互增强**，纯端到端还在爬功能安全的坡。你面试时能把这个 trade-off 讲清楚，比背出某个 SOTA 模型的名字有用得多。

## 怎么评价一个预测器好不好

简单过一下指标，你要能张口就来：

- **ADE（Average Displacement Error）**：预测轨迹与真值轨迹在所有时刻的平均欧氏距离。
- **FDE（Final Displacement Error）**：只看终点（horizon 末端）的误差。因为误差随时间累积，FDE 通常比 ADE 大得多。
- **minADE_k / minFDE_k**：多模态专用——输出 `k` 条轨迹，只取**最接近真值的那一条**算误差。这才公平：你猜了直行/左转/右转三条，真值是左转，就该拿左转那条来评，不能因为你"多猜了两条"就罚你。
- **Miss Rate（漏报率）**：终点误差超过某阈值（比如 2 米）算一次 miss。这个指标最接近安全——**漏报就是没预判到**。

但请记住本章开头那句：这些指标是给论文和横向对比用的。**上车真正的考核是"规划用了你的预测后，有没有出现该刹没刹、该躲没躲"**——那是场景级、闭环的评测，属于第九部分仿真的范畴。

## 小结

- 预测的任务是从**历史 + 地图 + 邻居**推断障碍物**多模态**的未来轨迹（每条带概率），horizon 通常 3~8 秒。
- 它难在**意图不可观测、参与者之间博弈、危险场景长尾稀有**——所以工程哲学是"不追求猜对，追求不猜漏"。
- 方法演进：**物理模型（CV/CTRV）**→**基于地图的车道跟随**→**机动意图分类**→**学习方法（LSTM/Transformer/VectorNet）**，一层比一层更懂上下文。我们给了 CV/CTRV 和车道跟随的可运行 Rust 实现。
- 物理模型至今是**保底基线**：永不发散、可做兜底与校验，是功能安全的底线。
- 多模态输出要做**概率归一化、模态剪枝、不确定性建模、时序一致性**；剪枝阈值对危险模态要格外小心，别让概率一票否决安全。
- 预测和规划本质**耦合**（鸡生蛋问题），强交互场景需要**条件/联合预测或博弈建模**；模块化 vs 端到端是当下的架构主战场。

下一章是这一部分最硬的一块——**规划**。我们要把"该怎么走"从一个模糊的愿望，变成 A\*、五次多项式、Frenet 坐标系这些能真的算出一条轨迹的数学与代码。系好安全带。
