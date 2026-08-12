# 7.2 横向与纵向控制：PID 到 MPC

上一节我们造好了车辆模型和一个前向仿真器。现在规划模块递过来一条轨迹（还记得 1.3 节的 `TrajectoryPoint` 吗），车上装着实时的 `VehicleState`。控制器要回答一个每 10~20 毫秒就要重答一次的问题：**这一瞬间，方向盘、油门、刹车各给多少，才能让车贴着这条线跑？**

这就是本节的全部内容。我们会从最朴素的 PID 出发，一路爬到量产高端方案的 MPC。这是一条清晰的能力阶梯，每一级都比上一级更强、也更贵。走完这条阶梯，你就摸到了控制工程师的核心手艺。

## 控制目标：把跟踪误差压到零

先把问题定义清楚。控制的输入是两样东西：

1. **参考轨迹（reference trajectory）**：规划给的 `Vec<TrajectoryPoint>`，每个点有期望的 `(x, y, v, heading)`。
2. **当前车辆状态**：定位/里程计给的 `VehicleState`（`x, y, yaw, v`）。

控制的输出是 1.3 节定义的 `ControlCommand`：

```rust
#[derive(Debug, Clone, Copy)]
pub struct ControlCommand {
    pub steering_angle: f64, // 方向盘转角（弧度）
    pub throttle: f64,       // 油门 0~1
    pub brake: f64,          // 刹车 0~1
}
```

习惯上我们把控制**解耦（decouple）**成两路，各管一摊：

- **纵向控制（longitudinal control）**：管**速度**。让车的实际速度 `v` 跟上轨迹要求的速度。输出油门/刹车。误差是 `速度误差 = v_ref - v`。
- **横向控制（lateral control）**：管**转向**。让车贴住轨迹这条线，不偏、不画龙。输出方向盘转角。误差主要是两个：**横向偏差（cross-track error）**——车离参考线有多远；**航向误差（heading error）**——车头方向和轨迹切线方向差多少。

> **中高级视角**：纵横解耦是个**工程近似**。真实车上纵向和横向是耦合的——急刹车时载荷前移、抓地力变化，会影响转向；高速转弯时向心力又吃掉纵向抓地力（"摩擦圆"）。低速、常规工况下解耦几乎无损，写起来也清爽；只有 MPC 这类方法才有能力把纵横耦合、连同约束一起统一优化。所以你会看到：**入门和大部分量产横向用解耦 + 几何/PID，追求极限性能才上耦合 MPC。**

轨迹跟踪的第一步永远是**找最近点 / 找参考点**——在参考轨迹上找到"车现在应该对标哪个点"。这是所有控制器的公共前置操作：

```rust,ignore
use crate::TrajectoryPoint; // 1.3 节的结构

/// 在轨迹上找离车当前位置最近的点的索引。
/// 生产代码会维护上一次的索引、只在附近小窗口搜索（O(1) 而非 O(n)），
/// 并处理"车已经开过轨迹末端"的情况。这里给最直白的版本。
pub fn nearest_index(traj: &[TrajectoryPoint], x: f64, y: f64) -> usize {
    traj.iter()
        .enumerate()
        .map(|(i, p)| {
            let dx = p.x - x;
            let dy = p.y - y;
            (i, dx * dx + dy * dy)
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0)
}
```

好，前置就绪。开始爬阶梯。

---

## PID：控制世界的"锤子"

PID 是控制界的螺丝刀——不精致，但能拧一半的螺丝。它不需要车辆模型，只盯着一个数字：**误差**。原理一句话：**看误差的现在（P）、过去（I）、和将来趋势（D），三者加权求和当作输出。**

设误差 `e(t) = 期望值 - 实际值`，PID 输出：

$$ u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\, d\tau + K_d\, \frac{de(t)}{dt} $$

三项各司其职，这是你必须建立的直觉：

- **比例项 P（Proportional）** `Kp·e`：误差越大，纠正越猛。它是主力。但**光有 P 会有稳态误差**——比如上坡定速巡航，需要持续给油门才能维持速度，可一旦车速追上、误差归零，P 项输出也归零，于是车又掉速……最终车会停在"差一点点"的地方僵持。这个差不掉的余量就是**稳态误差（steady-state error）**。
- **积分项 I（Integral）** `Ki·∫e`：把历史误差**累加**起来。只要还有一丁点残余误差，积分就持续攒、输出持续涨，直到把稳态误差彻底抹平。它专治 P 的稳态误差（比如自动补偿那个未知坡道）。代价是引入滞后、容易超调和振荡。
- **微分项 D（Derivative）** `Kd·ė`：看误差**变化的快慢**。误差在飞快缩小时，D 项踩一脚刹车，预判性地抑制超调，让响应更"稳"。代价是**对噪声极其敏感**——传感器抖一下，微分就炸一下。

> **面试题**：只用 P 控制器为什么会有稳态误差？加 I 为什么能消除？
> **答**：P 的输出正比于当前误差，要维持一个非零的输出（对抗重力/坡道/阻力等常值扰动），就必须保留一个非零误差——这就是稳态误差。I 项对误差**积分**，只要误差不为零，积分值就一直变化、输出一直调整，系统唯一能稳下来的状态就是误差=0。所以 I 能消除常值扰动下的稳态误差。

### 离散化：连续公式落到代码

车上是采样系统，每隔 `dt` 算一次。把积分换成累加、微分换成差分：

$$ u_k = K_p e_k + K_i \sum_{j} e_j\, dt + K_d \frac{e_k - e_{k-1}}{dt} $$

### 工程细节：教科书公式直接抄会翻车

裸 PID 公式在真车上必然出问题，四个坑必须堵：

1. **积分饱和（integral windup）与 anti-windup**：假设你猛踩加速，但油门已经到 100%（执行器饱和），车还是没到目标速度，于是积分项**持续疯狂累加**。等车终于提速、误差反号，那个攒了一大坨的积分值还得慢慢"放"出去，导致巨大超调。解法叫 **anti-windup**：执行器饱和时**别再往积分里加**（或做条件积分/反算）。这是 PID 工程化第一课。
2. **输出限幅（clamping）**：油门物理上就是 0~1，转角有机械极限。输出必须夹紧。
3. **微分噪声**：`(e_k - e_{k-1})/dt` 会放大测量噪声。常见处理：对微分项**低通滤波**，或改用"对测量值微分"（derivative on measurement）而非对误差微分，避免参考值突变时 D 项打出一个尖峰（derivative kick）。
4. **积分项初始化 / 复位**：控制器接管（从人工切自动）时要做**无扰切换（bumpless transfer）**，否则积分残值会让车猛地一窜。

下面是一个**生产级**的离散 PID，把上面的坑都堵了：

```rust
/// 一个工程化的离散 PID 控制器
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub out_min: f64, // 输出下限（如刹车侧 -1.0）
    pub out_max: f64, // 输出上限（如油门侧 +1.0）
    integral: f64,    // 积分累加器
    prev_meas: f64,   // 上一次的“测量值”（用于对测量微分）
    d_filtered: f64,  // 滤波后的微分项
    d_alpha: f64,     // 微分低通系数 0~1，越小越平滑
    initialized: bool,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, out_min: f64, out_max: f64) -> Self {
        Pid {
            kp, ki, kd, out_min, out_max,
            integral: 0.0,
            prev_meas: 0.0,
            d_filtered: 0.0,
            d_alpha: 0.1,
            initialized: false,
        }
    }

    /// setpoint: 期望值；measurement: 实际测量值；dt: 采样周期（秒）
    pub fn update(&mut self, setpoint: f64, measurement: f64, dt: f64) -> f64 {
        let error = setpoint - measurement;

        // --- P 项 ---
        let p = self.kp * error;

        // --- D 项：对“测量值”微分（避免 setpoint 突变引起的 derivative kick）---
        // d(error)/dt = -d(measurement)/dt （当 setpoint 恒定时）
        let d_raw = if self.initialized {
            -(measurement - self.prev_meas) / dt
        } else {
            0.0
        };
        // 低通滤波抑制噪声
        self.d_filtered = self.d_alpha * d_raw + (1.0 - self.d_alpha) * self.d_filtered;
        let d = self.kd * self.d_filtered;

        // --- I 项 + anti-windup（条件积分）---
        // 先算不含新积分增量的输出，判断是否会饱和
        let output_wo_new_i = p + self.ki * self.integral + d;
        // 仅当输出未顶到限位、或积分方向会让输出“往回走”时，才继续积分
        let will_saturate_high = output_wo_new_i >= self.out_max && error > 0.0;
        let will_saturate_low = output_wo_new_i <= self.out_min && error < 0.0;
        if !will_saturate_high && !will_saturate_low {
            self.integral += error * dt;
        }
        let i = self.ki * self.integral;

        self.prev_meas = measurement;
        self.initialized = true;

        // --- 输出限幅 ---
        (p + i + d).clamp(self.out_min, self.out_max)
    }

    /// 无扰切换 / 复位：接管时调用，防止积分残值造成突跳
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.d_filtered = 0.0;
        self.initialized = false;
    }
}
```

### 用 PID 做纵向速度控制

纵向控制是 PID 的经典战场：误差就是标量 `v_ref - v`，一维、直观、无耦合。把 PID 输出的"控制量"映射到油门/刹车：正的推油门，负的踩刹车（中间留个小死区防止油门刹车反复横跳）。

```rust,ignore
/// 纵向控制：PID(速度) → ControlCommand 的油门/刹车字段
pub fn longitudinal_control(
    pid: &mut Pid,
    v_ref: f64,
    v_actual: f64,
    dt: f64,
) -> (f64, f64) {
    // PID 输出范围设成 [-1, 1]：正=加速需求，负=制动需求
    let u = pid.update(v_ref, v_actual, dt);
    let deadband = 0.05;
    if u > deadband {
        (u, 0.0) // throttle, brake
    } else if u < -deadband {
        (0.0, -u)
    } else {
        (0.0, 0.0)
    }
}
```

这里 I 项就是我们在 7.1 埋的伏笔：**它自动补偿了未知坡道**。上坡时车掉速、误差持续为正、积分攒起来、油门自动加大，你根本不用知道坡度是多少。这就是反馈控制的美妙——**它不需要理解扰动，只需要盯着结果纠偏**。

> **真实项目里**：纵向控制常做成 **前馈 + 反馈** 结构：前馈项直接用 7.1 的纵向模型算"这个目标加速度大概要多少油门"（查标定表），把车快速推到大致正确的开度；PID 反馈只负责补前馈没算准的那点残差。纯反馈响应慢（要等误差出现才动作），前馈+反馈又快又稳，是量产标配。

PID 能不能做横向？能，误差取横向偏差就行。但它有个先天缺陷：**PID 只看误差，不懂车的运动学**。它不知道"方向盘转角要靠车速和轴距换算成曲率"，所以低速调好的增益一到高速就振荡。横向控制因此发展出一批**懂几何、懂模型**的专用方法。下面登场。

---

## Pure Pursuit（纯跟踪）：像追胡萝卜一样开车

Pure Pursuit（纯跟踪）是最优雅的横向几何控制器。直觉是这样：**在前方轨迹上选一个"前视点（look-ahead point）"，然后让车画一段圆弧，正好经过这个点。** 就像驴子追挂在前面的胡萝卜——永远盯着前方一个点开过去，自然就跟上了线。

### 几何推导

设车的参考点为后轴中心（呼应 7.1 的选择），当前朝向 `yaw`。在轨迹上找一个距车 `Ld`（前视距离 look-ahead distance）的目标点 `(gx, gy)`。把这个目标点变换到**车体坐标系**（车头为 x 轴、左侧为 y 轴），得到局部坐标 `(x_l, y_l)`，其中 `y_l` 就是目标点在车左右方向的偏移，`α` 是目标点相对车头的方位角。

现在的问题：车要走一段怎样的圆弧才能从当前位置（后轴中心，车头朝前）恰好到达 `(gx, gy)`？

设这段圆弧半径为 `R`。由几何（圆的弦与半径关系，或直接用正弦定理）可推出：目标点在车体系下，横向偏移 `y_l = Ld · sin(α)`。而经过原点、初始切线沿车头方向、到达偏移 `y_l` 处、弦长为 `Ld` 的圆弧，其半径满足：

$$ 2 R \sin(\alpha) = L_d \quad\Rightarrow\quad R = \frac{L_d}{2\sin(\alpha)} $$

于是曲率：

$$ \kappa = \frac{1}{R} = \frac{2\sin(\alpha)}{L_d} = \frac{2\, y_l}{L_d^2} $$

（最后一步用了 `y_l = Ld·sin(α)`。）再把曲率通过 7.1 的运动学关系 `κ = tan(δ)/L` 换成前轮转角，得到 **Pure Pursuit 的核心公式**：

$$ \delta = \arctan\!\left( \frac{2 L \sin(\alpha)}{L_d} \right) $$

其中 `L` 是轴距、`α` 是车头到前视点的夹角、`Ld` 是前视距离。就这么一个 `arctan`，横向控制就有了。

### 前视距离：这个控制器的灵魂

`Ld` 是 Pure Pursuit 唯一的调参旋钮，也是它的全部脾气：

- `Ld` **太小**：车紧盯着近处的点，反应激进，容易在直线上左右**画龙振荡**。
- `Ld` **太大**：车看得太远，转弯时"抄近道"（切弯 corner-cutting），跟踪精度差、过弯压线。

杀手锏是让 `Ld` **随车速变化**：`Ld = k · v + Ld_min`。高速看远（平稳），低速看近（精准）。这个 `k` 是速度增益。这其实就是我们后面要讲的**增益调度（gain scheduling）**的一个特例。

### Rust 实现

```rust,ignore
use crate::{TrajectoryPoint, VehicleState};

pub struct PurePursuit {
    pub wheelbase: f64,  // L
    pub k_ld: f64,       // 前视距离速度增益
    pub ld_min: f64,     // 最小前视距离
    pub max_steer: f64,
}

impl PurePursuit {
    /// 返回前轮转角 δ
    pub fn steer(&self, state: &VehicleState, traj: &[TrajectoryPoint]) -> f64 {
        let ld = self.k_ld * state.v + self.ld_min;

        // 从最近点往前找第一个距离 >= ld 的点作为前视点（goal）
        let near = crate::nearest_index(traj, state.x, state.y);
        let goal = traj[near..]
            .iter()
            .find(|p| {
                let dx = p.x - state.x;
                let dy = p.y - state.y;
                (dx * dx + dy * dy).sqrt() >= ld
            })
            .unwrap_or_else(|| traj.last().unwrap());

        // 目标点相对车头的方位角 α
        let dx = goal.x - state.x;
        let dy = goal.y - state.y;
        // 把 (dx,dy) 转到车体系，α = 目标点方向 - 车头方向
        let alpha = crate::normalize_angle(dy.atan2(dx) - state.yaw);

        // 实际前视距离（用真实到目标点的距离更稳）
        let ld_actual = (dx * dx + dy * dy).sqrt().max(self.ld_min);

        let delta = (2.0 * self.wheelbase * alpha.sin() / ld_actual).atan();
        delta.clamp(-self.max_steer, self.max_steer)
    }
}
```

> **中高级视角**：Pure Pursuit 只用了**位置**信息（车在哪、目标点在哪），完全没用**航向误差**。这让它实现极简、非常鲁棒，是很多低速/园区/农机/AGV 场景的首选。但它有个已知毛病：**恒定曲率弯道上会有稳态横向偏差**（因为它总在"抄近道"）。要消这个偏差，要么调 `Ld`，要么换下面这个把航向误差也用上的 Stanley。

---

## Stanley 控制器：斯坦福的夺冠方案

Stanley 控制器因斯坦福大学的 "Stanley" 车赢得 2005 DARPA 无人车挑战赛而得名。它比 Pure Pursuit 更聪明的地方：**同时盯着两个误差**——横向偏差和航向误差——并且参考点取**前轴**而非后轴。

### 两个误差项

Stanley 的前轮转角由两部分相加：

$$ \delta = \underbrace{\theta_e}_{\text{航向误差项}} + \underbrace{\arctan\!\left(\frac{k \cdot e_{fa}}{v}\right)}_{\text{横向偏差项}} $$

- **航向误差 `θ_e`（heading error）**：参考轨迹在最近点的切线方向 减去 车头朝向 `yaw`。这一项让车头**对齐**轨迹方向。（务必 `normalize_angle`！）
- **横向偏差 `e_fa`（cross-track error at front axle）**：**前轴中心**到参考线的垂直距离，带符号（车在线左还是线右）。这一项让车**贴回**线上。`k` 是横向增益。
- 除以 `v`：这是 Stanley 的精髓。**车速越高，同样的横向偏差对应的转角越小**，避免高速时因为一点点偏差就猛打方向。低速时 `v` 小、这一项大、纠偏果断。

直觉：航向项管"方向对不对"，横向项管"位置正不正"，`arctan` 保证转角有界（不会因偏差巨大而算出无穷转角），`/v` 做了自动的速度自适应。

### 关键工程细节

- **参考点取前轴**：`(x_f, y_f) = (x + L·cos(yaw), y + L·sin(yaw))`。用前轴算横向偏差是 Stanley 收敛性证明的前提，别用后轴。
- **低速奇异**：`v → 0` 时 `k·e/v` 爆炸。实现里分母加个小 `ε`（软化速度 `v + ε`）或低速下限。
- **横向偏差的符号**：用叉积判断车在参考线的左侧还是右侧，符号错了就变成正反馈，车直接冲出去。

### Rust 实现

```rust,ignore
pub struct Stanley {
    pub wheelbase: f64,
    pub k: f64,        // 横向增益
    pub k_soft: f64,   // 低速软化项（加到分母 v 上）
    pub max_steer: f64,
}

impl Stanley {
    pub fn steer(&self, state: &VehicleState, traj: &[TrajectoryPoint]) -> f64 {
        // 前轴中心
        let fx = state.x + self.wheelbase * state.yaw.cos();
        let fy = state.y + self.wheelbase * state.yaw.sin();

        // 以前轴为基准找最近点
        let idx = crate::nearest_index(traj, fx, fy);
        let p = &traj[idx];
        // 用相邻点估计该处轨迹切线方向（路径航向）
        let path_yaw = if idx + 1 < traj.len() {
            let n = &traj[idx + 1];
            (n.y - p.y).atan2(n.x - p.x)
        } else {
            p.heading
        };

        // 航向误差
        let theta_e = crate::normalize_angle(path_yaw - state.yaw);

        // 横向偏差（带符号）：前轴到最近点的向量，投影到路径法向
        let dx = fx - p.x;
        let dy = fy - p.y;
        // 叉积决定符号：路径切线 × 偏差向量
        let cross = path_yaw.cos() * dy - path_yaw.sin() * dx;
        let e_fa = cross; // 已带符号（左正右负，取决于坐标约定）

        // 横向偏差项，分母做低速软化
        let cross_term = (self.k * e_fa / (state.v + self.k_soft)).atan();

        let delta = theta_e + cross_term;
        delta.clamp(-self.max_steer, self.max_steer)
    }
}
```

> **面试题**：Pure Pursuit 和 Stanley 的核心区别是什么，各自适合什么场景？
> **答**：Pure Pursuit 只用位置、参考点取后轴、靠前视距离"抄近道"跟踪，实现简单、鲁棒，适合低速、路径平滑的场景，但弯道有稳态偏差。Stanley 同时用横向偏差 + 航向误差、参考点取前轴、带速度自适应，弯道跟踪更准、收敛性有理论保证，是中高速轨迹跟踪的常用基线。两者都不显式处理约束和舒适性，那是 MPC 的活。

---

## LQR：最优控制的第一步

到这里，PID/Pure Pursuit/Stanley 都是"手工设计规则 + 调增益"。**LQR（Linear Quadratic Regulator，线性二次型调节器）**换了个思路：**别手调增益了，让数学帮你算出最优增益。**

### 思路

LQR 需要两样东西：

1. 一个**线性状态空间模型**：`x_{k+1} = A·x_k + B·u_k`。横向控制里，状态 `x` 通常取 `[横向偏差, 横向偏差变化率, 航向误差, 航向误差变化率]`，把 7.1 的车辆模型在参考轨迹附近**线性化**得到 `A, B`。
2. 一个**代价函数（cost function）**，量化"我在乎什么"：

$$ J = \sum_{k=0}^{\infty} \left( x_k^\top Q\, x_k + u_k^\top R\, u_k \right) $$

- `Q`（半正定矩阵）：惩罚**状态误差**。`Q` 里横向偏差那一项调大，就是告诉优化器"我特别不能容忍偏离车道"。
- `R`（正定矩阵）：惩罚**控制量**。`R` 调大，就是"少打方向、开得柔和点"（省执行器、更舒适）。
- `Q` 和 `R` 的**比值**决定了控制器的性格：激进（贴线）还是温柔（平顺）。**这就是 LQR 的调参——从调增益变成了调"我在乎什么"，更符合直觉。**

### 求解

LQR 的漂亮之处：这个无穷时域最优化问题有**解析解**。最优控制律是状态的线性反馈：

$$ u_k = -K\, x_k $$

其中增益矩阵 `K` 由**代数黎卡提方程（Algebraic Riccati Equation, ARE）**解出：

$$ K = (R + B^\top P B)^{-1} B^\top P A $$

`P` 是黎卡提方程的解（通常迭代求解，反复迭代 `P` 直到收敛）。你不用手推黎卡提方程，但要知道：**给定 A, B, Q, R，一个求解器就能吐出最优增益 K，之后每一步控制只是一个矩阵乘法 `u = -K·x`，极快。**

```rust,ignore
// 用 nalgebra 求解离散 LQR（迭代解黎卡提方程）
// Cargo.toml: nalgebra = "0.33"
use nalgebra::DMatrix;

/// 迭代求解离散时间代数黎卡提方程，返回最优反馈增益 K，使 u = -K x 最优。
pub fn dlqr(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    iters: usize,
) -> DMatrix<f64> {
    let mut p = q.clone();
    for _ in 0..iters {
        let bt_p = b.transpose() * &p;
        // R + Bᵀ P B
        let inner = r + &bt_p * b;
        let inner_inv = inner.clone().try_inverse().expect("R+BᵀPB 不可逆");
        // 黎卡提迭代： P = Q + Aᵀ P A - Aᵀ P B (R+BᵀPB)⁻¹ Bᵀ P A
        let at_p = a.transpose() * &p;
        p = q + &at_p * a - &at_p * b * inner_inv * &bt_p * a;
    }
    // K = (R + Bᵀ P B)⁻¹ Bᵀ P A
    let bt_p = b.transpose() * &p;
    let inner = r + &bt_p * b;
    inner.try_inverse().unwrap() * &bt_p * a
}
```

> **中高级视角**：LQR 是 Apollo 早期横向控制器的核心（`LatController`）。它的软肋是**处理不了约束**——它假装转角可以无限大、加速度可以任意，然后靠 `R` 和事后限幅去"劝住"。真车上转角有极限、加速度有舒适性上限、前方有障碍物边界，这些都是**硬约束**。LQR 表达不了"绝对不能超过 X"。谁能？下面的压轴选手。

---

## MPC：现代高端方案的皇冠

**MPC（Model Predictive Control，模型预测控制）**是控制阶梯的顶端，也是当下高端量产和 Robotaxi 横向/纵向控制的主流。理解了前面所有铺垫，MPC 就是水到渠成的一句话：

> **每一个控制周期，用车辆模型往未来预测一小段（预测时域），求解一个带约束的最优化问题，找出未来 N 步的最优控制序列，只执行第一步，下一周期用最新状态重新算一遍。**

这个"算一长串、只走第一步、下周期重算"的机制叫**滚动时域优化（receding horizon control）**。为什么只执行第一步就重算？因为世界在变（新的障碍物、模型误差、扰动），与其死守一个几秒前算好的长计划，不如每一步都拿最新信息重新规划——**永远基于当下的最优决策**。

### MPC 凭什么碾压前面所有方法

1. **它会"预判"**：PID/Stanley 都是"看着当前误差纠偏"的反应式控制。MPC 用模型**向前看**——它知道"前面 2 秒有个急弯"，会提前减速、提前打方向。这叫**前瞻性（preview）**，是平顺性的关键。
2. **它原生处理约束（constraints）**：这是 MPC 相对 LQR 的决定性优势。转角极限、转角变化率（打方向别太猛）、加速度舒适性上限、车道边界、甚至和障碍物的安全距离——全都能作为**硬约束**写进优化问题。求解器保证解一定满足。
3. **它统一处理纵横耦合和多目标**：跟踪精度、舒适性（惩罚 jerk 冲击度）、控制能耗，全塞进一个代价函数，一次优化全兼顾。

### 数学骨架

在每个控制周期，MPC 求解这个优化问题：

$$
\min_{u_0, \dots, u_{N-1}} \; \sum_{k=0}^{N-1} \Big( \|x_k - x_k^{ref}\|_Q^2 + \|u_k\|_R^2 \Big) + \|x_N - x_N^{ref}\|_{Q_f}^2
$$

约束条件：

$$
\begin{aligned}
&x_{k+1} = f(x_k, u_k) &&\text{(车辆模型，7.1 那套)} \\
&u_{min} \le u_k \le u_{max} &&\text{(执行器极限)} \\
&\Delta u_{min} \le u_k - u_{k-1} \le \Delta u_{max} &&\text{(变化率约束，保平顺)} \\
&x_k \in \mathcal{X} &&\text{(状态约束：车道边界、安全距离)}
\end{aligned}
$$

- `N` 是**预测时域（prediction horizon）**步数，比如 20 步 × 0.1 秒 = 向前看 2 秒。
- 目标函数和 LQR 长得像（二次型），但这里是**有限时域 + 带约束**。
- 若模型 `f` 线性化、约束线性，这就是个**二次规划（QP, Quadratic Program）**问题——有成熟高效的求解器，能在毫秒级解完。若用非线性模型，则是 NLP，更慢，常用于低频规划层。

### 和优化求解器的关系

MPC 工程师干的活，一大半是**把控制问题"翻译"成求解器认识的标准形式（QP/NLP），然后调求解器**。Rust 生态里：

- **OSQP**（有 Rust 绑定 `osqp`）：算子分裂 QP 求解器，工业界 MPC 的热门选择，快且鲁棒。
- **Clarabel**（纯 Rust 写的锥优化求解器，`clarabel` crate）：现代、活跃、纯 Rust，非常适合在车载 Rust 栈里用。
- 也有人用 `nlopt`、`good_lp` 建模层等。

### 结构性 Rust 骨架

下面不是完整可跑的 MPC（那需要接一个 QP 求解器并搭建矩阵，篇幅太长），而是把 MPC 的**控制流骨架**写清楚，让你看懂它每一拍在干什么：

```rust,ignore
/// MPC 横向+纵向控制器的骨架（结构演示，非完整实现）
pub struct Mpc {
    pub horizon: usize,   // 预测时域 N
    pub dt: f64,          // 每步时长
    // 权重矩阵 Q, R, Qf；约束上下限等（略）
    // solver: 某个 QP 求解器句柄，如 osqp::Problem
}

pub struct MpcState {
    pub x: f64, pub y: f64, pub yaw: f64, pub v: f64,
}

impl Mpc {
    /// 每个控制周期调用一次，返回“只执行第一步”的控制指令
    pub fn solve(
        &mut self,
        state: &MpcState,
        reference: &[TrajectoryPoint], // 未来 N 步的参考
    ) -> ControlInput {
        // 1) 在参考轨迹附近，把车辆模型（7.1 的 f）线性化，
        //    得到每一步的 A_k, B_k（离散、随参考点变化的时变线性化）。
        let (a_mats, b_mats) = self.linearize_along_reference(state, reference);

        // 2) 把“预测时域内的动力学 + 代价 + 约束”组装成一个标准 QP：
        //      min  (1/2) zᵀ H z + fᵀ z
        //      s.t. lb ≤ A_c z ≤ ub
        //    其中决策变量 z 打包了未来 N 步的状态和控制。
        let qp = self.build_qp(state, reference, &a_mats, &b_mats);

        // 3) 调求解器解这个 QP（osqp / clarabel）。
        //    实时性关键：热启动（warm start，用上周期解做初值）+ 迭代上限，
        //    保证在控制周期内一定返回一个“够好”的解。
        let solution = self.solve_qp(&qp);

        // 4) 只取第一步控制量执行，其余丢弃，下周期用最新 state 重算。
        ControlInput {
            steer: solution.u0_steer,
            accel: solution.u0_accel,
        }
    }

    // 下面这些是真正的工作量所在，此处仅列签名示意
    fn linearize_along_reference(&self, _s: &MpcState, _r: &[TrajectoryPoint])
        -> (Vec<[[f64; 4]; 4]>, Vec<[[f64; 2]; 4]>) { unimplemented!() }
    fn build_qp(&self, _s: &MpcState, _r: &[TrajectoryPoint],
                _a: &[[[f64;4];4]], _b: &[[[f64;2];4]]) -> QpProblem { unimplemented!() }
    fn solve_qp(&mut self, _qp: &QpProblem) -> MpcSolution { unimplemented!() }
}

struct QpProblem;
struct MpcSolution { u0_steer: f64, u0_accel: f64 }
```

> **面试题**：MPC 为什么"算了未来 N 步却只执行第一步"，这不是浪费吗？
> **答**：不浪费，这正是它鲁棒的来源。模型有误差、有扰动、环境在变，几秒前算的长序列后面几步早已不准。每周期用最新状态重算（滚动时域），等于把"计划"不断校正，永远基于当前真实状态给出最优的下一步。算 N 步是为了让"这一步"考虑到未来的约束和目标（前瞻），执行一步是为了不被过时的计划绑架。这是"长远考虑，脚踏实地"的控制版本。

> **中高级视角**：别神化 MPC。它**吃算力**（每周期解一个优化问题）、**依赖模型精度**（模型错了预测就错）、**依赖求解器可靠性**（求解失败或超时怎么办？必须有兜底——退回上一步解，或降级到 Stanley/PID）。量产 MPC 工程一半的功夫花在"求解器在最坏情况下也能按时返回一个安全解"上。**能不能保证实时性和失败兜底，是 MPC 从 demo 走向量产的分水岭。**

---

## 工程现实：让控制器活在真车上

算法只是骨架，下面这些"脏活"才决定车开起来是丝滑还是顿挫。这也是控制工程师日常真正在搞的事。

### 执行器延迟与补偿

从"控制器输出转角指令"到"前轮真的转到那个角度"，中间有**延迟（latency）**——线控转向、液压制动都有几十到上百毫秒的滞后。控制器若无视延迟，就是在用"过时的世界"做决策，高速时足以引发振荡。

补偿办法：用 7.1 的车辆模型，把当前状态**向前预测**一个延迟时长 `τ`（"等指令生效时，车会在哪"），拿这个**预测状态**去算控制。这叫**延迟补偿（latency compensation）**，几乎是量产控制器的标配。

```rust,ignore
/// 用运动学模型把状态向前预测 τ 秒，补偿执行器延迟
fn predict_forward(state: &VehicleState, last_u: &ControlInput, wheelbase: f64, tau: f64) -> VehicleState {
    // 用上一拍实际执行的控制量，推进 τ 秒（可细分几小步提高精度）
    step(state, last_u, &VehicleParams::passenger_car(), tau) // 复用 7.1 的 step
}
```

### 增益调度（gain scheduling）

同一套增益不可能通吃所有车速。低速要灵敏、高速要稳重。**增益调度**就是让增益随工况（主要是车速）变化——Pure Pursuit 的 `Ld = k·v + Ld_min`、Stanley 的 `/v`、以及一张"车速 → PID 增益"的插值表，都是增益调度。量产系统里，一张按车速分档的增益表是标配。

```rust
/// 按车速线性插值 PID 增益（增益调度的最简形式）
fn schedule_gain(v: f64, low: (f64, f64, f64), high: (f64, f64, f64), v_lo: f64, v_hi: f64) -> (f64, f64, f64) {
    let t = ((v - v_lo) / (v_hi - v_lo)).clamp(0.0, 1.0);
    (
        low.0 + t * (high.0 - low.0),
        low.1 + t * (high.1 - low.1),
        low.2 + t * (high.2 - low.2),
    )
}
```

### 控制频率

横向/纵向控制器通常跑 **50~100 Hz**（周期 10~20 ms），比感知（10 Hz）快得多。原因：控制直接关系车身稳定，必须高频闭环才能压住扰动、跟上车速。频率不够，高速时两拍之间车已经跑出去大半米，控制精度和稳定性都保不住。MPC 因为解优化耗时，有时跑得稍慢（比如 20~50 Hz），中间用插值或低级控制器补齐。

### 标定与平顺性

- **标定（calibration）**：所有增益、前视距离、Q/R 权重、油门刹车映射表，都要在实车/仿真上一遍遍刷。这是控制落地最耗时的环节，没有捷径。
- **平顺性（comfort）**：乘客不只在乎"跟没跟上线"，更在乎**加加速度（jerk，加速度的变化率）**。急加急减、方向盘猛打都会让人晕车。工程上通过**限制 jerk**、对控制指令做**低通滤波/速率限制**、在 MPC 代价里**惩罚 Δu** 来保证平顺。一个跟踪误差稍大但丝般顺滑的控制器，体验往往胜过一个死贴线但顿挫的。

> **真实项目里**：控制模块的代码量里，纯算法可能只占三成，剩下七成是：延迟补偿、增益调度表、各种限幅和速率限制、传感器异常时的降级逻辑（定位丢了怎么办、轨迹为空怎么办）、平顺性滤波、与底盘协议的对接。**能把这七成做扎实，才是控制工程师和"会写 PID 的人"的区别。** 顺带一提，你工作环境里 `inf/` 那套把 Python 推理流水线移植成 Rust 的 SDK，正是"Rust 在智驾数据闭环里已落地"的例子——控制器最终也会走上类似的工程化、可回放、可测试的道路。

## 小结

- 控制的目标是**把跟踪误差压到零**，工程上解耦成**纵向（管速度）**和**横向（管转向）**两路。
- **PID** 不需模型、只看误差（P 现在、I 过去、D 将来），是纵向速度控制的主力；工程化必须处理 **anti-windup、限幅、微分噪声、无扰切换**；I 项自动补偿未知坡道是它的精髓。
- **Pure Pursuit** 用前视点几何，核心 `δ = arctan(2L·sin α / Ld)`，简单鲁棒，靠前视距离 `Ld` 调性格，弯道有稳态偏差。
- **Stanley** 同时用横向偏差 + 航向误差、参考点取前轴、`/v` 做速度自适应，弯道更准、收敛有保证。
- **LQR** 让数学算出最优增益 `u=-Kx`，把"调增益"变成"调 Q/R 权重"，但**处理不了硬约束**。
- **MPC** 是皇冠：滚动时域优化、用车辆模型预判、原生处理约束和纵横耦合，是现代高端方案的主流；代价是吃算力、依赖模型和求解器、必须有实时性保证和失败兜底。
- **工程现实**决定成败：执行器延迟补偿、增益调度、50~100 Hz 控制频率、标定、以及为平顺性限制 jerk——这七成"脏活"才是控制工程师的真本事。

到这里，第七部分"控制"就讲完了——你已经能让车从"看见世界"一路走到"精确地操作方向盘和油门"。下一部分，我们换个维度：这一切模块如何通过**中间件**在多进程、多频率下真正协同运转起来，让 Rust 写的节点在车上跑成一张活的系统图。
