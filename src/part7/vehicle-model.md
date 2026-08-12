# 7.1 车辆运动学与动力学模型

前面六个部分，我们让车"看见了"世界（感知）、"知道了"自己在哪（定位）、"预判了"别人（预测）、"决定了"自己怎么走（规划）。现在，规划模块递给你一条漂亮的轨迹——一串 `TrajectoryPoint`，每个点告诉你"在时刻 t，我希望车在 (x, y)、以速度 v、朝向 heading"。你的任务，就是把方向盘、油门、刹车拧到合适的量，让这台一吨半的铁疙瘩**真的**沿着这条线跑。

这就是控制。而控制的第一课，不是控制算法，是**车辆模型（vehicle model）**。

## 为什么控制离不开车辆模型

先讲个直觉。假设你要控制一个点，"我让它往左它就往左，我让它停它就停"——这叫**全向（holonomic）**系统，比如屏幕上的鼠标光标。控制它几乎不需要"模型"，误差多少就补多少。

但车不是。车是**非完整约束（nonholonomic）**系统：它**不能横着走**。你想让车往左平移半米，对不起，车只能"打方向盘 + 往前开"，走一段弧线才能挪过去。方向盘转角和车的实际位移之间，隔着一层**运动学关系**。你要是不懂这层关系，控制器就只能瞎试——误差大了猛打方向，结果画龙（振荡）。

> **中高级视角**：控制的本质是**求逆**。规划告诉你"我要这个结果（轨迹）"，车辆模型描述"给定输入会产生什么结果"，控制器要做的是把这个模型**反过来**——"想要这个结果，我该给什么输入"。模型越准，这个"求逆"越靠谱。所以模型不是可有可无的理论装饰，它是控制器脑子里那张"车会怎么动"的地图。

车辆模型有三个层次，复杂度递增：

1. **运动学模型（kinematic model）**：只讲几何——方向盘转角、车速、轴距如何决定车的位置和朝向变化。**不管质量、不管力**。低速（城市工况大部分时间）下足够准，是本书和绝大多数横向控制器的主力。
2. **动力学模型（dynamic model）**：引入质量、转动惯量、**轮胎侧向力**。高速、大侧向加速度（急转弯、湿滑路面）下，轮胎会"打滑"，运动学假设失效，必须上动力学。
3. **纵向模型**：油门/刹车 → 驱动力/制动力 → 加速度，还要扣掉空气阻力、滚动阻力、坡道分量。

这一节我们把这三层讲透，最后给你一个**可运行的前向仿真器**——输入一串控制指令，输出一条轨迹。这个仿真器不仅是理解模型的工具，它就是第九部分仿真章的"车辆动力学内核"的雏形。

## 坐标与状态量

先把"车此刻是什么状态"用几个数说清楚。在一个平面（俯视，忽略上下坡和车身姿态）里，车的状态是：

- `x`, `y`：车某个参考点在世界（或地图）坐标系下的位置，单位米。参考点通常取**后轴中心**（运动学模型里这个选择让公式最干净，下面会看到为什么）。
- `yaw`（也写作 `θ` 或 `ψ`）：车头朝向角（heading / yaw angle），弧度。约定 `x` 轴正方向为 0，逆时针为正。
- `v`：车速，米/秒。运动学模型里通常指后轴中心沿车身纵轴方向的速度。

用 Rust 表达（我们复用第一部分 `Pose` 的精神，但控制里习惯用一个更紧凑的平面状态）：

```rust
/// 车辆的平面运动状态（bicycle model 用）
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleState {
    pub x: f64,     // 后轴中心 x（米）
    pub y: f64,     // 后轴中心 y（米）
    pub yaw: f64,   // 航向角（弧度）
    pub v: f64,     // 纵向速度（米/秒）
}
```

控制输入（control input）是我们能拧的旋钮。运动学自行车模型里是两个：

- `delta`（`δ`）：前轮**转向角（steering angle）**，弧度。注意这是**前轮**相对车身纵轴的角度，不是方向盘转角——两者差一个方向盘传动比（steering ratio，通常 15:1 左右），标定时要换算。
- `a`：纵向加速度（米/秒²），由油门/刹车产生。或者有些模型直接把 `v` 当输入（"速度指令"），取决于你的执行器接口。

```rust
/// 运动学自行车模型的控制输入
#[derive(Debug, Clone, Copy)]
pub struct ControlInput {
    pub steer: f64,  // 前轮转向角 δ（弧度）
    pub accel: f64,  // 纵向加速度 a（米/秒²）
}
```

## 运动学自行车模型：完整推导

"自行车模型（bicycle model）"这个名字是个绝妙的类比：一辆四轮车，左右两个前轮合并成一个"虚拟前轮"，左右两个后轮合并成一个"虚拟后轮"，于是四轮车塌缩成一辆**自行车**——前轮可转向，后轮固定朝向车身。这个简化在直线和缓转弯下几乎无损，却把公式砍到能手推的程度。

### 几何设定

想象俯视图：

```text
              前轮(可转 δ)
                 O  ← 前轴中心 F
                /|
               / |
              /  | L (轴距 wheelbase)
             /   |
            /    |
           O-----+  ← 后轴中心 R  (我们的参考点 (x,y))
        后轮      车身纵轴方向 = yaw
```

关键参数是**轴距 `L`（wheelbase）**：前轴中心到后轴中心的距离。

核心假设（这是运动学模型成立的根基，务必记住）：**轮胎不打滑，每个轮子只沿它自己朝向的方向滚动，没有侧向滑移。** 于是：

- 后轮朝向 = 车身朝向 `yaw`，所以后轴中心的速度方向就是 `yaw`。
- 前轮朝向 = `yaw + δ`，前轴中心的速度方向是 `yaw + δ`。

### 从"轮子不侧滑"推出转弯半径

既然两个轮子都不侧滑，它们的速度方向就分别垂直于各自到"瞬时转动中心（ICR, Instantaneous Center of Rotation）"的连线。也就是说，整车绕某个点 ICR 做纯转动。

设后轴中心到 ICR 的距离为 `R`（这就是后轴的转弯半径）。从几何上看（后轴速度垂直于 R，前轴速度方向偏了 `δ`），前轴、后轴、ICR 构成一个直角三角形，直角在后轴处：

$$ \tan(\delta) = \frac{L}{R} $$

用文字说：**前轮转角的正切，等于轴距除以后轴转弯半径。** 转角越大，`tan δ` 越大，`R` 越小——转得越急。这就是你打满方向盘车能原地画小圈、方向盘回正车走直线（`R → ∞`）的数学。

由此得到**曲率（curvature）`κ`**——曲率就是转弯半径的倒数，是规划和控制里反复出现的量：

$$ \kappa = \frac{1}{R} = \frac{\tan(\delta)}{L} $$

**这个式子是横向控制的命根子**：规划给你一条带曲率的轨迹，你反过来用 `δ = atan(L · κ)` 就能算出"要贴着这条线走，前轮该转多少度"。后面 Pure Pursuit、Stanley 全都绕着它转。

### 状态的时间导数（连续形式）

后轴中心以速度 `v` 沿 `yaw` 方向前进，所以位置变化：

$$ \dot{x} = v \cos(yaw), \qquad \dot{y} = v \sin(yaw) $$

朝向变化率 = 角速度。整车绕 ICR 转，角速度 = 线速度 / 半径：

$$ \dot{yaw} = \frac{v}{R} = v \cdot \kappa = \frac{v \tan(\delta)}{L} $$

速度变化 = 加速度输入：

$$ \dot{v} = a $$

把四个凑一起，就是**运动学自行车模型的状态方程**（以后轴为参考点）：

$$
\begin{aligned}
\dot{x} &= v \cos(yaw) \\
\dot{y} &= v \sin(yaw) \\
\dot{yaw} &= \frac{v}{L} \tan(\delta) \\
\dot{v} &= a
\end{aligned}
$$

四行公式，请把它们刻进脑子。整个横向控制、整个车辆仿真，都是从这四行长出来的。

> **面试题**：为什么运动学自行车模型的参考点常取后轴中心而不是质心？
> **答**：取后轴中心时，速度方向恰好等于车身朝向 `yaw`，`ẋ = v·cos(yaw)` 干净利落。若取质心，车身还有一个"侧滑角 β（sideslip angle）"，速度方向 = `yaw + β`，公式多一项，`β` 又依赖 `δ`，推导变复杂。取后轴是"选坐标让方程最简"的经典操作。当然也有以质心为参考点的版本（Apollo 等用），换汤不换药。

### 阿克曼转向：那个"虚拟前轮"是怎么来的

前面把左右前轮合并成一个虚拟前轮，转角 `δ`。但真实的车左右前轮**转角并不相等**——这就是**阿克曼转向几何（Ackermann steering geometry）**。

原因很直接：转弯时，内侧前轮离 ICR 近、转弯半径小，外侧前轮离 ICR 远、半径大。要让两个前轮都"不侧滑"（都指向各自的圆弧切线），内轮必须比外轮转得更多。设左右轮距（轮距 track width）为 `T`，则理想阿克曼下：

$$ \cot(\delta_{outer}) - \cot(\delta_{inner}) = \frac{T}{L} $$

那个"虚拟前轮转角 `δ`"，就是把内外轮平均到轴中线上的等效转角，满足 `tan(δ) = L / R`。

工程上你通常**不需要**自己算内外轮——转向机构（转向梯形）在硬件上近似实现了阿克曼，你的控制器只管输出等效 `δ`，底盘去分配。但你得**知道它存在**：低速大转角泊车时阿克曼误差最明显，某些高精度泊车控制会显式建模它。

```rust
/// 由等效前轮转角 δ 反推左右前轮转角（理想阿克曼）
/// track: 左右前轮轮距 T；wheelbase: 轴距 L
fn ackermann_split(delta: f64, wheelbase: f64, track: f64) -> (f64, f64) {
    if delta.abs() < 1e-6 {
        return (0.0, 0.0); // 直行
    }
    let r = wheelbase / delta.tan(); // 后轴转弯半径（带符号）
    // 内外轮的转弯半径差半个轮距
    let inner = (wheelbase / (r - track / 2.0)).atan();
    let outer = (wheelbase / (r + track / 2.0)).atan();
    (inner, outer)
}
```

## 离散化：把微分方程变成"仿真一步"

计算机不会解微分方程，它只会一小步一小步地往前算。给定当前状态和输入，"往前推进 `dt` 秒"就叫**积分（integration）**或**状态递推**。最简单的是**前向欧拉（forward Euler）**：`下一个 = 当前 + 导数 × dt`。

$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos(yaw_k)\, dt \\
y_{k+1} &= y_k + v_k \sin(yaw_k)\, dt \\
yaw_{k+1} &= yaw_k + \frac{v_k}{L}\tan(\delta_k)\, dt \\
v_{k+1} &= v_k + a_k\, dt
\end{aligned}
$$

欧拉法简单，但 `dt` 大或车速高时误差可观（尤其 `yaw` 累积误差会让轨迹"漂"）。工程上两种改良很常见：

1. **半隐式 / 先更新朝向再更新位置**：先算 `yaw_{k+1}`，用更新后的 `yaw` 或中点 `yaw` 去推位置，精度明显提升，几乎零成本。
2. **龙格-库塔 4 阶（RK4）**：采样导数四次加权平均，精度高一个数量级，仿真器里常用。控制器实时算一步时，为省算力多用欧拉或半隐式。

下面代码把这几种都实现了，方便你对比。

```rust,ignore
impl VehicleState {
    /// 前向欧拉一步
    pub fn step_euler(&self, u: &ControlInput, wheelbase: f64, dt: f64) -> VehicleState {
        VehicleState {
            x: self.x + self.v * self.yaw.cos() * dt,
            y: self.y + self.v * self.yaw.sin() * dt,
            yaw: self.yaw + self.v / wheelbase * u.steer.tan() * dt,
            v: self.v + u.accel * dt,
        }
    }

    /// 半隐式欧拉：先更新 v 和 yaw，再用更新后的值推位置。
    /// 几乎零额外成本，轨迹精度明显更好，是仿真的推荐默认。
    pub fn step_semi_implicit(&self, u: &ControlInput, wheelbase: f64, dt: f64) -> VehicleState {
        let v_next = self.v + u.accel * dt;
        let yaw_next = self.yaw + v_next / wheelbase * u.steer.tan() * dt;
        // 用中点朝向推进位置，进一步降低误差
        let yaw_mid = 0.5 * (self.yaw + yaw_next);
        VehicleState {
            x: self.x + v_next * yaw_mid.cos() * dt,
            y: self.y + v_next * yaw_mid.sin() * dt,
            yaw: normalize_angle(yaw_next),
            v: v_next,
        }
    }
}

/// 把角度规整到 (-π, π]，防止 yaw 无限累积导致后续 cos/sin 精度下降。
pub fn normalize_angle(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut a = a % two_pi;
    if a > std::f64::consts::PI {
        a -= two_pi;
    } else if a <= -std::f64::consts::PI {
        a += two_pi;
    }
    a
}
```

> **真实项目里**：`normalize_angle` 这种"角度环绕"处理是横向控制 bug 的头号来源。车在 `yaw = 179°` 微微右转到 `-179°`，如果你不规整，误差算出来是 `358°` 而不是 `2°`，控制器会疯狂反打方向。凡是涉及角度差的地方（航向误差、yaw 误差），**永远**先规整到 `(-π, π]`。这个坑本节先埋下，7.2 的 Stanley 里会正面撞上。

## 动力学自行车模型：当轮胎开始打滑

运动学模型有个致命假设：**轮胎不侧滑**。低速下这没问题——轮胎的侧向抓地力绰绰有余，车指哪走哪。但当你**高速过弯**，需要的**向心力**（`m·v²/R`）大到轮胎抓地力扛不住时，轮胎会产生**侧偏（slip）**：轮子的实际运动方向和它指向的方向不一致，两者夹角叫**侧偏角（slip angle）`α`**。此时运动学模型会系统性地低估你需要的转角，车会"推头"（转向不足）冲出弯道。

**什么时候必须上动力学模型？** 一个粗略的判据是**侧向加速度**：`a_lat = v² · κ = v² / R`。当 `a_lat` 超过约 `0.4g`（≈ 4 m/s²）时，轮胎进入非线性区，运动学模型开始明显失真。城市低速工况（`< 40 km/h`、缓转弯）几乎用不到；高速公路变道、赛道、湿滑路面则必须用。

动力学自行车模型引入这些新角色：

- `m`：整车质量（kg）。
- `Iz`：绕竖直轴（yaw 轴）的**转动惯量（moment of inertia）**，决定车头"甩起来"有多难。
- `lf`, `lr`：质心到前轴、后轴的距离（`lf + lr = L`）。参考点这次取**质心**。
- `Cf`, `Cr`：前后轮的**侧偏刚度（cornering stiffness）**，即"每单位侧偏角能产生多少侧向力"，单位 N/rad。线性轮胎模型下，侧向力 `Fy = C · α`。
- 状态量除了 `x, y, yaw`，还要加**侧向速度 `vy`**（或侧滑角 β）和**横摆角速度（yaw rate）`r = yaw_dot`**。

线性动力学自行车模型（小侧偏角假设）的核心两个方程（侧向 + 横摆）大致长这样：

$$
\begin{aligned}
m(\dot{v_y} + v_x r) &= F_{yf}\cos\delta + F_{yr} \\
I_z \dot{r} &= l_f F_{yf}\cos\delta - l_r F_{yr}
\end{aligned}
$$

其中前后轮侧向力（线性轮胎）：

$$
F_{yf} = C_f\,\alpha_f, \quad \alpha_f = \delta - \frac{v_y + l_f r}{v_x}, \qquad
F_{yr} = C_r\,\alpha_r, \quad \alpha_r = -\frac{v_y - l_r r}{v_x}
$$

不必现在就把这堆符号背下来。你要带走的**核心认知**是：

1. 动力学模型把"车"从一个几何点，升级成一个**有质量、有惯性、靠轮胎摩擦力转弯**的物理系统。
2. 它多了 `vy`、`r` 两个状态，多了 `m, Iz, Cf, Cr` 一堆需要**标定**的参数——而 `Cf, Cr` 随载重、胎压、路面变化，标不准动力学模型反而不如运动学模型稳。
3. 注意方程里有除以 `vx`（纵向车速）——**车速趋近 0 时公式奇异（分母爆炸）**。所以实践中常见做法是：**低速用运动学、高速用动力学**，中间做平滑切换。这是很多量产横向控制器的真实结构。

> **中高级视角**：MPC（7.2 会讲）里到底用运动学还是动力学模型做预测，是个经典权衡。运动学模型线性化后 QP 求解快、鲁棒、低速稳；动力学模型高速精准但参数敏感、低速奇异、算力更贵。很多量产方案是"运动学 MPC + 速度自适应的增益"，只有主打高速/激烈驾驶的才上动力学 MPC。别一上来就迷信更复杂的模型——**能用简单模型稳定跑起来的，才是好工程**。

## 纵向模型：油门刹车到底给了多少加速度

前面把纵向输入直接当成加速度 `a`，很方便。但真实执行器给的是**油门开度**和**刹车压力**，中间隔着一整套动力总成。纵向动力学的牛顿第二定律是：

$$ m\,\dot{v} = F_{drive} - F_{brake} - F_{aero} - F_{roll} - F_{grade} $$

逐项拆解：

- **驱动力 `F_drive`**：油门开度经过发动机/电机 → 变速箱 → 车轮，转成牵引力。电车近似线性、响应快；油车有发动机 MAP、换挡、涡轮迟滞，非线性且有延迟。
- **制动力 `F_brake`**：刹车踏板/压力 → 制动力矩。也有液压建立时间（几十到上百毫秒的延迟）。
- **空气阻力 `F_aero`** = `0.5 · ρ · Cd · A · v²`。注意它**与速度平方成正比**——120 km/h 时的空气阻力是 60 km/h 时的**四倍**，这是高速定速巡航为什么费油/费电的直接原因。`ρ` 空气密度、`Cd` 风阻系数、`A` 迎风面积。
- **滚动阻力 `F_roll`** = `Crr · m · g · cos(θ_slope)`，`Crr` 滚动阻力系数（约 0.01~0.015），近似为常数。
- **坡道分量 `F_grade`** = `m · g · sin(θ_slope)`。上坡为正（拖后腿），下坡为负（帮你加速）。**坡道是纵向控制的隐形杀手**：同样的油门，上坡车速掉、下坡车速冲，纯前馈会跟不上，所以纵向控制器几乎都要靠反馈（PID 的积分项）去补这个未知坡道——7.2 会详谈。

一个够用的纵向模型 Rust 实现：

```rust
/// 纵向车辆参数
pub struct LongitudinalParams {
    pub mass: f64,        // m, kg
    pub cd_a: f64,        // Cd * A, 风阻系数×迎风面积 (m²)
    pub crr: f64,         // 滚动阻力系数
    pub air_density: f64, // ρ, kg/m³, 约 1.225
    pub gravity: f64,     // g, 9.81
}

impl LongitudinalParams {
    /// 给定当前车速、驱动力/制动力、坡道角，算纵向加速度
    /// drive_force / brake_force 单位 N；slope 单位弧度（上坡为正）
    pub fn accel(&self, v: f64, drive_force: f64, brake_force: f64, slope: f64) -> f64 {
        let f_aero = 0.5 * self.air_density * self.cd_a * v * v * v.signum();
        let f_roll = self.crr * self.mass * self.gravity * slope.cos();
        let f_grade = self.mass * self.gravity * slope.sin();
        let net = drive_force - brake_force - f_aero - f_roll - f_grade;
        net / self.mass
    }
}
```

> **真实项目里**：从"我想要 2 m/s² 加速度"到"油门踩百分之多少"这一步，量产车上常常不是靠一个公式，而是靠一张**标定表（calibration map）**：在各种车速、各种目标加速度下实车测出对应的油门/刹车开度，做成查找表 + 插值。这张表由标定工程师在测试场一格一格刷出来，是控制"接地气"的部分——再漂亮的算法，标定不到位一样顿挫。

## 一个可运行的前向仿真器

理论讲完，上硬货。下面是一个**完整、可编译运行**的运动学自行车模型前向仿真器：输入一串控制指令序列，输出整条轨迹。它就是后面控制器和仿真章的"数字沙盘"——你可以把任何控制器接上去，看车怎么跑。

`Cargo.toml`（无第三方依赖，纯标准库）：

```toml
[package]
name = "bicycle_sim"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "bicycle_sim"
path = "src/main.rs"
```

`src/main.rs`：

```rust
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ControlInput {
    pub steer: f64, // 前轮转角 δ（弧度）
    pub accel: f64, // 纵向加速度 a（m/s²）
}

/// 把角度规整到 (-π, π]
pub fn normalize_angle(a: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let mut a = a % two_pi;
    if a > PI {
        a -= two_pi;
    } else if a <= -PI {
        a += two_pi;
    }
    a
}

/// 车辆物理参数
pub struct VehicleParams {
    pub wheelbase: f64,   // 轴距 L（米）
    pub max_steer: f64,   // 前轮最大转角（弧度）
    pub max_accel: f64,   // 最大加速度（m/s²）
    pub max_decel: f64,   // 最大减速度（m/s²，取正值）
}

impl VehicleParams {
    /// 一辆典型乘用车的默认参数
    pub fn passenger_car() -> Self {
        VehicleParams {
            wheelbase: 2.7,
            max_steer: 0.6, // ≈ 34°
            max_accel: 3.0,
            max_decel: 6.0,
        }
    }
}

/// 半隐式欧拉推进一步，输入先按物理限制夹紧（clamp）——
/// 真实执行器不可能瞬间给出无穷大的转角/加速度。
pub fn step(state: &VehicleState, u: &ControlInput, p: &VehicleParams, dt: f64) -> VehicleState {
    let steer = u.steer.clamp(-p.max_steer, p.max_steer);
    let accel = u.accel.clamp(-p.max_decel, p.max_accel);

    let mut v_next = state.v + accel * dt;
    if v_next < 0.0 {
        v_next = 0.0; // 简化：不建模倒车
    }
    let yaw_rate = v_next / p.wheelbase * steer.tan();
    let yaw_next = state.yaw + yaw_rate * dt;
    let yaw_mid = 0.5 * (state.yaw + yaw_next);

    VehicleState {
        x: state.x + v_next * yaw_mid.cos() * dt,
        y: state.y + v_next * yaw_mid.sin() * dt,
        yaw: normalize_angle(yaw_next),
        v: v_next,
    }
}

/// 前向仿真：给定初始状态和控制序列，返回整条轨迹（含初始点）。
pub fn simulate(
    init: VehicleState,
    inputs: &[ControlInput],
    p: &VehicleParams,
    dt: f64,
) -> Vec<VehicleState> {
    let mut traj = Vec::with_capacity(inputs.len() + 1);
    traj.push(init);
    let mut s = init;
    for u in inputs {
        s = step(&s, u, p, dt);
        traj.push(s);
    }
    traj
}

fn main() {
    let p = VehicleParams::passenger_car();
    let dt = 0.05; // 20 Hz

    // 初始：原点、朝 +x、静止
    let init = VehicleState { x: 0.0, y: 0.0, yaw: 0.0, v: 0.0 };

    // 造一段控制序列：先直线加速 2 秒，再保持恒定左转 3 秒，最后松油门轻刹 1 秒
    let mut inputs = Vec::new();
    let steps_per_sec = (1.0 / dt) as usize;
    for _ in 0..(2 * steps_per_sec) {
        inputs.push(ControlInput { steer: 0.0, accel: 2.0 }); // 加速直行
    }
    for _ in 0..(3 * steps_per_sec) {
        inputs.push(ControlInput { steer: 0.15, accel: 0.0 }); // 恒定左转弧线
    }
    for _ in 0..(1 * steps_per_sec) {
        inputs.push(ControlInput { steer: 0.0, accel: -3.0 }); // 刹车回正
    }

    let traj = simulate(init, &inputs, &p, dt);

    // 每 0.5 秒打印一次，观察车怎么跑
    println!("{:>6} {:>8} {:>8} {:>8} {:>7}", "t(s)", "x", "y", "yaw°", "v");
    for (i, s) in traj.iter().enumerate() {
        if i % (steps_per_sec / 2) == 0 {
            println!(
                "{:6.2} {:8.2} {:8.2} {:8.1} {:7.2}",
                i as f64 * dt,
                s.x,
                s.y,
                s.yaw.to_degrees(),
                s.v
            );
        }
    }

    // 验证曲率关系：恒定 δ=0.15、L=2.7 时，转弯半径应约为 L/tan(δ)
    let r = p.wheelbase / 0.15_f64.tan();
    println!("\n恒定 δ=0.15 时理论转弯半径 R ≈ {:.2} m", r);
}
```

运行 `cargo run` 你会看到车先沿 x 轴加速，然后拐出一条平滑的左转弧线，最后减速。把打印的 `(x, y)` 拿去画个图，就是一条清晰的轨迹。你还能验证曲率公式：恒定 `δ=0.15`、`L=2.7`，理论半径 `R = 2.7 / tan(0.15) ≈ 17.8 m`，把弧线段的点拟合个圆，半径就在这附近。

**这个仿真器的价值**：7.2 讲的每一个横向/纵向控制器，都可以插到这个 `simulate` 循环里——把固定的 `inputs` 换成"控制器根据当前 `state` 和参考轨迹实时算出的 `ControlInput`"，你就有了一个完整的**跟踪仿真闭环**，能亲眼看着 PID、Pure Pursuit、Stanley 各自怎么把车"拽"回轨迹线上。这正是下一节的舞台。

## 小结

- 控制离不开车辆模型，因为车是**非完整约束**系统（不能横走），方向盘和位移之间隔着运动学关系；控制的本质是拿模型**求逆**。
- **运动学自行车模型**从"轮胎不侧滑"这一条假设，推出核心关系 `tan(δ) = L/R`、曲率 `κ = tan(δ)/L`，以及四行状态方程 `ẋ=v·cosθ, ẏ=v·sinθ, θ̇=v·tanδ/L, v̇=a`——这是横向控制的地基。
- **阿克曼几何**解释了左右前轮转角为何不等；工程上底盘硬件近似实现它，控制器只输出等效 `δ`。
- **动力学模型**在高侧向加速度（`> 0.4g`）、高速、湿滑时才必要，引入质量、转动惯量、轮胎侧偏力，代价是参数难标、低速奇异——**能用运动学稳跑就别硬上动力学**。
- **纵向模型**里，空气阻力随 `v²` 增长、坡道是隐形杀手，从加速度到油门开度量产上常靠**标定表**。
- 我们写了一个**可运行的前向仿真器**，它是下一节所有控制器的试验台。

下一节，我们让车真正"跟线"——从最朴素的 PID，一路讲到量产高端方案的 MPC，把这一节的模型变成方向盘上的实际动作。
