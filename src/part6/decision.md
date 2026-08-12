# 6.3 决策与行为规划

上一章我们学会了怎么**算**一条轨迹——A\*、五次多项式、Frenet。但我一直偷偷绕过一个更根本的问题：算轨迹之前，得先知道**要算哪一种**轨迹。是跟着前车走，还是变道超过去？是在行人前停下让行，还是加速通过？红灯到底停不停？

这些问题的答案不是一个连续的数值，而是一个**离散的选择**。求解它们的这一层，叫**决策（decision making）** 或**行为规划（behavior planning）**。它坐在预测和运动规划之间：拿着预测给的"别人要干嘛"，输出一个离散意图和一组约束，交给上一章的运动规划去落成具体轨迹。

这一章我们把决策讲透，并在最后用一个**红绿灯 + 行人**的完整 Rust 决策器，把 1.3 节那个悬了整本书的横穿行人彻底了结。

## 决策到底在决什么：离散的动作空间

先建立直觉。运动规划的输出空间是**连续**的（无穷多条可能轨迹），而决策的输出空间是**离散**的——一小把互斥的高层动作：

- **跟车（follow / car-following）**：锁定前车，保持安全距离，速度随它。
- **超车（overtake）**：变道绕过慢车再回来。
- **让行（yield）**：给有路权的一方让路（行人、对向左转车、主路车流）。
- **停车（stop）**：在停止线、红灯、障碍前停下。
- **变道（lane change）**：向左或向右换到相邻车道。
- **保持巡航（cruise）**：无干扰，按目标速度前进。

决策的工作就是**在每个规划周期，根据当前场景，从这些动作里选一个（或一个组合），并生成对应的约束**——比如选了"让行行人"，就往运动规划里塞一个"在行人前 X 米处速度必须为 0"的约束，运动规划再去算那条平滑减速到停的轨迹。

**决策和运动规划的接口**，本质是这样一组约束：

```rust,ignore
/// 决策层输出给运动规划的"意图 + 约束"
#[derive(Debug, Clone)]
pub struct Decision {
    pub intent: DrivingIntent,        // 离散意图
    pub target_speed: f64,            // 期望巡航速度上限 (m/s)
    pub stop_point_s: Option<f64>,    // 若需停车，停止点的纵向位置 (Frenet s)
    pub follow_target_id: Option<u32>, // 若跟车，锁定的目标障碍物 id
    pub target_lane: LaneId,          // 目标车道（变道时不同于当前车道）
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrivingIntent {
    Cruise,
    Follow,
    Overtake,
    Yield,
    Stop,
    LaneChangeLeft,
    LaneChangeRight,
}
```

看清楚这个接口：决策**不画轨迹**，它只给"停在哪、跟着谁、目标多快、去哪条道"这些**离散选择和标量约束**。轨迹的活儿是上一章运动规划的。这个职责切分是模块化架构的精髓——决策关心"做什么（what）"，运动规划关心"怎么做（how）"。

## 方法一：有限状态机（FSM）

最经典、最直观、也最"够用"的决策方法是**有限状态机（Finite State Machine, FSM）**。DARPA Urban Challenge 那批开山之作，决策层清一色是 FSM。它的模型简单到一句话：**系统在任意时刻处于有限个"状态"之一，根据事件/条件在状态间跳转，每个状态对应一套行为。**

比如一个简化的高速驾驶 FSM：`巡航 → (前方有慢车) → 跟车 → (左道空且值得超) → 变道 → (回到巡航速度) → 巡航`。

Rust 的 `enum` + `match` 是表达 FSM 的**完美工具**——编译器会强制你处理每个状态，漏一个都过不了编译。这是 Rust 相比 C++ 写决策的一个实在优势。看一个能跑的高速跟车/超车 FSM：

```rust,ignore
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriveState {
    Cruise,        // 自由巡航
    Follow,        // 跟车
    LaneChangeLeft // 向左变道超车
}

/// 决策所需的精简环境输入
pub struct Scene {
    pub ego_speed: f64,          // 自车速度
    pub desired_speed: f64,      // 期望巡航速度
    pub front_gap: Option<f64>,  // 前车距离 (None=前方无车)
    pub front_speed: f64,        // 前车速度
    pub left_lane_clear: bool,   // 左道是否安全可并
    pub back_to_lane: bool,      // 是否已完成变道回到目标车道
}

impl DriveState {
    /// 状态转移：输入当前状态和场景，输出下一个状态
    pub fn transition(self, s: &Scene) -> DriveState {
        const SAFE_GAP: f64 = 25.0; // 触发跟车的距离阈值
        match self {
            DriveState::Cruise => {
                // 前方出现慢车且距离进入安全阈值 -> 转跟车
                match s.front_gap {
                    Some(gap) if gap < SAFE_GAP && s.front_speed < s.desired_speed => {
                        DriveState::Follow
                    }
                    _ => DriveState::Cruise,
                }
            }
            DriveState::Follow => {
                // 前车走了/加速了 -> 回巡航；左道安全且前车确实慢 -> 变道超车
                match s.front_gap {
                    None => DriveState::Cruise,
                    Some(gap) if gap > SAFE_GAP * 1.5 => DriveState::Cruise, // 加滞回，防抖
                    _ if s.left_lane_clear && s.front_speed < s.desired_speed - 2.0 => {
                        DriveState::LaneChangeLeft
                    }
                    _ => DriveState::Follow,
                }
            }
            DriveState::LaneChangeLeft => {
                // 变道完成 -> 回巡航；否则继续变道
                if s.back_to_lane {
                    DriveState::Cruise
                } else {
                    DriveState::LaneChangeLeft
                }
            }
        }
    }

    /// 每个状态对应的决策输出
    pub fn action(self, s: &Scene) -> Decision {
        match self {
            DriveState::Cruise => Decision {
                intent: DrivingIntent::Cruise,
                target_speed: s.desired_speed,
                stop_point_s: None,
                follow_target_id: None,
                target_lane: LaneId::Current,
            },
            DriveState::Follow => Decision {
                intent: DrivingIntent::Follow,
                target_speed: s.front_speed, // 速度随前车
                stop_point_s: None,
                follow_target_id: Some(0),   // 实际填前车 id
                target_lane: LaneId::Current,
            },
            DriveState::LaneChangeLeft => Decision {
                intent: DrivingIntent::LaneChangeLeft,
                target_speed: s.desired_speed,
                stop_point_s: None,
                follow_target_id: None,
                target_lane: LaneId::Left,
            },
        }
    }
}
```

注意那个 `SAFE_GAP * 1.5` 的**滞回（hysteresis）**——进跟车的阈值和出跟车的阈值故意错开。这是 FSM 一个**必须掌握**的工程细节：如果进出用同一个阈值，当前车距离恰好在阈值附近抖动时，状态会疯狂地 `Cruise↔Follow` 来回跳，决策"神经质"，乘客感受是车一顿一顿的。加滞回（或加最小保持时间）是 FSM 防抖的标配。

### FSM 的致命局限：状态爆炸

FSM 简单可靠，但**扩展性极差**。问题叫**状态爆炸（state explosion）**。真实驾驶的状态不只是"巡航/跟车/变道"，还叠加着"红灯/绿灯""有行人/无行人""路口/直路""左转/直行"……这些维度是**组合**的。你想覆盖"跟车时遇到红灯，且右边有行人"，就得为每个维度组合造一个状态。维度一多，状态数和转移数**指数爆炸**——几十个状态、几百条转移边，没人维护得了，改一条边可能引发三处 bug。这是 FSM 在复杂城市场景（尤其路口）逐渐力不从心的根本原因。

> **面试题**：FSM 做自动驾驶决策的最大问题是什么，怎么缓解？
> **答**：状态爆炸——多个正交的情境维度组合起来状态数指数增长，难以维护和验证。缓解手段：① 分层状态机（HFSM，把状态分组、嵌套，正交维度用并行状态机而非笛卡尔积展开）；② 换用行为树（组合式、更易复用）；③ 对连续权衡的部分（选跟谁、变不变道）改用基于代价的方法，FSM 只管最顶层的粗粒度模式。

## 方法二：行为树（Behavior Tree）

为缓解状态爆炸，游戏 AI 和机器人界流行**行为树（Behavior Tree, BT）**。它不用"状态 + 转移"建模，而是用一棵**树**组织行为，每个 tick（周期）从根节点往下遍历，靠几种控制节点决定走哪个分支：

- **Sequence（顺序）**：依次执行子节点，全成功才成功，一个失败就失败（"与"逻辑）——比如"检查左道安全" → "打转向灯" → "执行变道"。
- **Selector / Fallback（选择）**：依次尝试子节点，一个成功就成功（"或"逻辑）——比如"尝试超车" 失败则 "尝试跟车" 失败则 "紧急停车"，天然表达优先级兜底。
- **Condition（条件）** 与 **Action（动作）**：叶子节点，判断条件或执行具体行为。

BT 相比 FSM 的核心优势是**组合性和可复用性**：子树可以像积木一样拼接、复用，加一个新行为通常是挂一棵新子树，不用像 FSM 那样重连一堆转移边。它的优先级结构（Selector 从左到右）也天然适合"优先超车，不行就跟车，再不行就停"这种带兜底的决策。缺点是**难以表达带记忆的、复杂时序**的逻辑（BT 默认每 tick 重新评估，做"坚持某个动作一段时间"不如 FSM 自然），且树一大同样会变得难读。工业界常见 **FSM + BT 混用**：顶层模式用 FSM，模式内部的行为编排用 BT。

## 方法三：基于代价/风险的决策

FSM 和 BT 本质都是**人写死的规则**。但很多决策不是"非黑即白"的规则能覆盖的——**要不要现在变道**取决于"当前车道多堵、目标车道多空、变道收益 vs 风险"的连续权衡。这类问题更适合**基于代价（cost-based）** 的决策：给每个候选动作算一个代价，选代价最小的。

这其实和上一章 lattice planner "撒候选、打分、选优"是一个哲学，只是这里的候选是**离散的高层动作**而非连续轨迹。给"是否变道"算代价的例子：

```rust,ignore
/// 对一个候选高层动作评估综合代价，越低越好
fn cost_of_action(action: DrivingIntent, s: &Scene) -> f64 {
    let mut cost = 0.0;
    match action {
        DrivingIntent::Follow => {
            // 跟车代价：被慢车拖累的"效率损失"
            cost += (s.desired_speed - s.front_speed).max(0.0) * 1.0;
        }
        DrivingIntent::LaneChangeLeft => {
            // 变道代价：换道的舒适/风险成本 + 目标道不安全的重罚
            cost += 3.0; // 变道本身的固定成本（乘客不喜欢频繁变道）
            if !s.left_lane_clear {
                cost += 1000.0; // 不安全就是天价，等于硬约束
            }
        }
        _ => cost += 5.0,
    }
    cost
}

/// 基于代价选出最优离散动作
fn decide_by_cost(s: &Scene) -> DrivingIntent {
    [DrivingIntent::Follow, DrivingIntent::LaneChangeLeft]
        .into_iter()
        .min_by(|a, b| cost_of_action(*a, s)
            .partial_cmp(&cost_of_action(*b, s)).unwrap())
        .unwrap()
}
```

注意那个 `1000.0` 的**软化硬约束**技巧——把"绝对不能做"的事编码成一个巨大代价，而不是单独写一堆 `if`。这样安全约束和效率权衡统一在同一个代价框架里比较，代码更整洁。风险（risk）也常这么建模：把碰撞概率 × 后果严重度作为代价项，让决策**显式地权衡收益与风险**。

## 方法四：博弈与交互——决策最难的部分

前面所有方法都有个隐含假设：**别人的行为是我决策的固定输入**。但 6.1 讲预测时那个"鸡生蛋"又回来了——在**强交互**场景，别人的行为**取决于我怎么决策**。

典型场景：**无保护左转（unprotected left turn）**。你要左转穿过对向车流，对向车会不会让你，取决于你有没有果断地往前探。你保守地等，对方也不会停下来让你，你可能永远等下去（机器人常见的"freezing robot"问题——过度保守到寸步难行）；你太激进又危险。这**不是一个预测问题，是一个博弈（game）问题**——双方的最优策略互相依赖。

建模交互决策的两大理论工具：

- **POMDP（部分可观测马尔可夫决策过程）**：把别人的**意图**当作**隐藏状态**（你观测不到，只能从行为推断），把决策建成"在意图不确定下最大化长期期望收益"的序贯决策问题。它数学上非常完备——自然地处理了不确定性、观测、长期规划。但**求解极其昂贵**（状态/信念空间连续且高维），在线实时求解基本靠近似（如 POMCP、宏动作、场景树），是学术界热点、工业界谨慎落地的方向。

- **博弈论（game theory）**：把交通参与者建成博弈的玩家，找纳什均衡（Nash equilibrium）之类的解概念——"在对方也理性应对的前提下，我的最优动作"。对建模"你进我退/你退我进"的耦合特别贴切，近年在无保护左转、匝道汇入的研究里很活跃。

> **中高级视角**：POMDP 和博弈论在论文里美如画，但你去问一线量产团队，会发现真正上车的多是**它们的工程化近似**：用几个离散的意图假设 + 条件预测（6.1 讲的），枚举"如果我让/我抢，对方大概率怎样"，再用代价选择——本质是把博弈**降维成有限几个可枚举的场景来评估**。完整的在线 POMDP 求解在几十毫秒预算里几乎不现实。能把"理论上的博弈"翻译成"工程上可算的近似"，是高级决策工程师的核心能力。

## 方法五：强化学习——很热，但上车有坎

既然决策是"序贯地在不确定环境里最大化收益"，那不正是**强化学习（Reinforcement Learning, RL）** 的定义吗？确实，RL 在决策上研究火热——尤其擅长那些规则难以穷举的交互场景（汇入、博弈），让智能体在仿真里自己"练"出策略。

但请保持清醒，RL 上量产车有几道**真实的坎**：

- **可解释性差**：神经网络策略是黑盒。出了事故，你没法像看 FSM 那样指着某条规则说"这里错了"。功能安全（ISO 26262）和事故追责都需要可解释，这是硬门槛。
- **安全保证难**：RL 策略在训练分布内表现好，遇到没见过的场景（长尾）可能给出灾难性动作，且没有形式化的安全下界。你很难向监管证明"它在所有情况下都不会做傻事"。
- **仿真到现实的鸿沟（sim-to-real gap）**：RL 靠海量试错训练，真车上不能试错（撞了就是事故），只能在仿真里练，但仿真和真实世界的差异会让策略打折。
- **奖励设计难**："安全、舒适、高效、守规"如何设计成一个标量奖励，本身就是老大难，设计不当会学出钻空子的怪异行为。

所以当下的务实做法是 **RL 打辅助，不当主决策**：用 RL 学**参数**（比如代价权重、跟车距离偏好）而非直接输出动作、用 RL 做**仿真里的对手**（生成刁难的交通流来测你的规则决策）、或在**受限的低风险子问题**上用 RL。让一个纯 RL 黑盒直接决定方向盘和刹车，量产车上目前还没人敢——**安全和可解释性是过不去的坎**。

> **真实项目里**：面试被问"你怎么看 RL 做决策"，别无脑吹也别无脑贬。正解是：RL 在**交互建模和长尾探索**上有独特价值，但**可解释性和安全保证**的短板决定了它现阶段是"规则决策的补充和研究工具"，而非替代。能把这个 trade-off 说清楚，比你会调某个 RL 算法更能体现工程成熟度。

## 场景化决策：为什么路口是"决策的珠峰"

直路上决策相对简单（跟车、变道就那么几样）。真正让决策工程师头秃的是**路口（intersection）** 类场景，因为它同时叠满了所有难点：

- **无保护左转**：前面说的博弈难题，要穿越对向车流。
- **汇入 / 匝道合流（merging）**：要在主路车流里找一个缝插进去，也是博弈——你得和主路车"协商"出一个 gap。
- **路口通行权**：红绿灯、停车让行标志、环岛、四向停车（谁先到谁先走）……规则本身就复杂，还得预测别人守不守规。
- **遮挡（occlusion）**：路口视线常被建筑、大车挡住，"看不见的地方可能冲出东西"，决策要对**未观测到的潜在风险**保守（比如盲区路口降速）。

这些场景没有银弹，工业界的做法是**场景化（scenario-based）**——识别出当前属于哪类场景（路口左转/汇入/跟车/……），切换到该场景专门设计和调优的决策逻辑。这也是为什么各家都在堆**场景库**：每一类难场景都是一套单独的决策 + 大量仿真回归。1.3 节那个行人横穿场景，就是最基础的一个;真实系统里有成百上千个。

## 收尾：红绿灯 + 行人的完整决策器

理论讲完，我们兑现承诺——用一个**能跑的** FSM 决策器，衔接 1.3 节那个场景：自车在城市道路直行，前方有红绿灯，路边一个行人可能横穿（预测给了多模态：横穿概率 0.7）。决策器要综合红绿灯状态和行人预测，输出"通过 / 让行 / 停车"的决策。

```rust,ignore
// Cargo.toml 无需额外依赖

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LightColor { Red, Yellow, Green }

/// 一条行人预测模态（来自 6.1 的预测模块）
#[derive(Debug, Clone, Copy)]
pub struct PedPrediction {
    pub crossing_prob: f64, // 横穿概率
    pub time_to_conflict: f64, // 预计到达自车路径的时间 (s)，越小越危险
}

/// 决策场景输入
pub struct IntersectionScene {
    pub ego_speed: f64,        // 自车速度 (m/s)
    pub dist_to_stopline: f64, // 到停止线距离 (m)
    pub light: LightColor,
    pub ped: Option<PedPrediction>, // 路边行人预测，None=无行人
}

/// 决策状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntersectionDecision {
    Proceed,     // 保持速度通过
    YieldToPed,  // 为行人减速让行
    StopAtLine,  // 在停止线停车（红灯）
}

/// 核心决策逻辑：安全优先级从高到低依次判断
pub fn decide(scene: &IntersectionScene) -> IntersectionDecision {
    // 舒适减速能停下所需的最小距离：v^2 / (2a)，取 a=2.5 m/s^2 舒适减速度
    const COMFORT_DECEL: f64 = 2.5;
    let stop_distance = scene.ego_speed.powi(2) / (2.0 * COMFORT_DECEL);

    // —— 优先级 1：行人风险（最高，安全一票否决）——
    if let Some(ped) = scene.ped {
        // 横穿概率高，且行人会在我到达冲突点前后进入路径 -> 让行
        // 注意：这里对"高危模态"用概率阈值，但阈值定得保守（0.3 就让）
        let ped_dangerous = ped.crossing_prob > 0.3 && ped.time_to_conflict < 4.0;
        if ped_dangerous {
            return IntersectionDecision::YieldToPed;
        }
    }

    // —— 优先级 2：红绿灯 ——
    match scene.light {
        LightColor::Red => IntersectionDecision::StopAtLine,
        LightColor::Yellow => {
            // 黄灯两难：太近了刹不住就通过，否则停
            // 这正是真实"黄灯困境区(dilemma zone)"的工程处理
            if scene.dist_to_stopline < stop_distance {
                IntersectionDecision::Proceed // 已进入无法安全停车的区域，通过更安全
            } else {
                IntersectionDecision::StopAtLine
            }
        }
        LightColor::Green => IntersectionDecision::Proceed,
    }
}

/// 把离散决策翻译成给运动规划的约束（对接上一章）
pub fn to_planning_constraint(
    d: IntersectionDecision,
    scene: &IntersectionScene,
) -> Decision {
    match d {
        IntersectionDecision::Proceed => Decision {
            intent: DrivingIntent::Cruise,
            target_speed: scene.ego_speed.max(8.0),
            stop_point_s: None,
            follow_target_id: None,
            target_lane: LaneId::Current,
        },
        IntersectionDecision::YieldToPed => Decision {
            intent: DrivingIntent::Yield,
            target_speed: 0.0,
            // 在冲突点前留出安全余量停车
            stop_point_s: Some(scene.dist_to_stopline - 2.0),
            follow_target_id: None,
            target_lane: LaneId::Current,
        },
        IntersectionDecision::StopAtLine => Decision {
            intent: DrivingIntent::Stop,
            target_speed: 0.0,
            stop_point_s: Some(scene.dist_to_stopline),
            follow_target_id: None,
            target_lane: LaneId::Current,
        },
    }
}

fn main() {
    // 场景：自车 10 m/s，距停止线 30 m，绿灯，但路边行人 0.7 概率横穿、2.5 秒后到冲突点
    let scene = IntersectionScene {
        ego_speed: 10.0,
        dist_to_stopline: 30.0,
        light: LightColor::Green,
        ped: Some(PedPrediction { crossing_prob: 0.7, time_to_conflict: 2.5 }),
    };
    let d = decide(&scene);
    println!("决策: {:?}", d); // -> YieldToPed：绿灯也要给横穿行人让行！
    let constraint = to_planning_constraint(d, &scene);
    println!("给规划的约束: {:?}", constraint);
}
```

跑出来的结果是 **`YieldToPed`**——**即使是绿灯，也要给高概率横穿的行人让行**。这正是 1.3 节那个场景的答案：感知看到行人的横向速度，预测算出 0.7 的横穿概率，决策让安全优先级压过绿灯的通行权，输出"减速让行"，再把"在行人前 2 米停下"的约束交给上一章的运动规划去生成那条平滑减速轨迹。**整条 感知→预测→决策→规划 的链路，在这个 60 行的例子里闭环了。**

请特别注意 `decide` 函数的结构——**它是按安全优先级从高到低短路判断的**：行人风险第一，红绿灯第二。这个"优先级 + 短路"结构是安全攸关决策的通用模式：**最危险的事最先判、能一票否决**。以及那个黄灯困境区的处理（刹不住就通过），是真实工程里必须面对的两难——教科书不讲，但路上天天遇到。

> **中高级视角**：这个 FSM 决策器看着简单，但它体现了几个决策工程的核心原则：① 安全优先级短路（行人 > 信号灯）；② 概率阈值对高危模态取保守值（0.3 就让，而非 0.5）；③ 用物理量（刹车距离）而非拍脑袋常数做判断（黄灯困境区）；④ 决策只出离散意图 + 标量约束，把轨迹留给规划。把这四条想明白，你写的决策就不是玩具，是能讲给功能安全评审听的东西。

## 小结

- 决策/行为规划坐在**预测和运动规划之间**，输出**离散意图 + 标量约束**（停在哪、跟谁、目标速度、去哪条道），不画轨迹——轨迹是运动规划的活儿。
- **FSM**：`enum + match` 天然表达，Rust 编译器帮你穷尽状态；必须懂**滞回防抖**；死穴是**状态爆炸**。
- **行为树**：组合式、可复用、优先级兜底，缓解状态爆炸；弱在复杂时序记忆。常与 FSM 混用。
- **基于代价/风险**：把离散动作打分选优，用"天价代价"软化硬约束，适合连续权衡（变不变道）。
- **博弈与交互**（POMDP、博弈论）：处理无保护左转、汇入这类"别人怎么做取决于我怎么做"的强交互；理论完备但实时求解昂贵，工业界多用**可枚举场景 + 条件预测**的近似。
- **强化学习**：在交互和长尾探索上有价值，但**可解释性、安全保证、sim-to-real** 三道坎决定它当前是"辅助和研究工具"，非主决策。
- **场景化**是应对复杂路口的现实路线：识别场景类型 → 切换专用决策逻辑 → 堆场景库 + 仿真回归。
- 我们用一个红绿灯 + 行人的 FSM 决策器闭环了 1.3 节的场景：**绿灯也要给横穿行人让行**，靠的是"安全优先级短路 + 高危模态保守阈值 + 物理量判断"。

第六部分到此结束——你现在能**预测**别人、**决策**自己要干嘛、并**规划**出一条能开的轨迹。但轨迹只是"愿望"，怎么让方向盘和油门精确地跟上它，是下一部分**控制**的主题。我们会从车辆的运动学/动力学模型讲起，一路做到 PID 和 MPC。轨迹画好了，该真的让车动起来了。
