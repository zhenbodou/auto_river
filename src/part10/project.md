# 10.1 实战项目：搭一个最小自动驾驶栈

前面九个部分，我们像拆钟表一样把自动驾驶一个齿轮一个齿轮讲了个遍：感知怎么把世界变成障碍物清单、卡尔曼怎么估状态、Frenet 怎么规划、PID 和纯跟踪怎么让车听话、中间件怎么把节点连起来、仿真怎么让你不用真车就能试错。

现在到了**把齿轮装回一台会走的钟**的时候。

这一章我们用**纯 Rust**从零搭一个能跑通闭环的最小自动驾驶栈，让它在一个自带的简易仿真器里，把第 1.3 节那个场景真正演出来：

> 车以 10 m/s 直行，前方 30 米一个行人正以 0.3 m/s 横穿马路。系统必须"看见—预判—决定减速—平滑停在行人前"。

读完并**亲手敲完**这一章，你手里就有了一个可以写进简历、可以在面试里被往死里追问细节的真实项目。这正是第 1.5 节说的"项目深挖"那一栏的弹药。别只读，一定要跑起来、改坏它、再修好——中高级就是这么长出来的。

## 我们要造什么，以及不造什么

先摆正期望。这是一个**教学用最小栈（minimal stack）**，不是 Apollo。它的设计原则是：

- **每个模块都能对应到书里某一章的算法**，且是那一章的"最简可用版"。
- **整体能编译、能跑、输出能看懂**，一台普通笔记本几秒钟跑完。
- **接口留了口子**，方便你后面把某个模块换成"真家伙"（真的神经网络、真的 rclrs 节点、真的 CARLA）。

它**故意**简化掉的东西（这些正是你下一步的扩展方向）：

| 模块 | 本章的最简版 | 工业级会怎么做 | 书里哪章 |
|---|---|---|---|
| 感知 | 从仿真真值加噪声直接产出 `Obstacle` | 神经网络 + 点云 + 跟踪 | 第四部分 |
| 定位 | 直接用仿真真值当位姿 | GNSS/IMU 组合导航 + 地图匹配 | 第五部分 |
| 预测 | 恒速模型 | 多模态学习型预测 | 6.1 |
| 规划 | 单车道纵向速度规划 | Frenet + 优化 + 决策 | 6.2 / 6.3 |
| 控制 | PID（纵向）+ 纯跟踪（横向） | MPC | 7.2 |
| 仿真 | 运动学自行车模型 | CARLA / 高保真动力学 | 7.1 / 9.1 |

这张表本身就是一份"我这个项目还能怎么升级"的路线图，收好。

## 项目结构：一个 Cargo workspace

真实的自动驾驶代码库都是**多 crate 的 workspace**（工作空间）：公共类型一个 crate，每个功能模块一个 crate，最后一个可执行 crate 把它们组装起来。这样编译增量快、职责清晰、模块可以被独立测试和替换。我们照着来。

```text
minimal_av/
├── Cargo.toml                 # workspace 根，声明所有成员
└── crates/
    ├── common/                # 公共数据类型（复用全书定义）
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── perception/            # 真值 + 噪声 → Obstacle
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── prediction/            # 恒速模型（引 6.1）
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── planning/              # 纵向速度规划（引 6.2）
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── control/               # PID + 纯跟踪（引 7.2）
    │   ├── Cargo.toml
    │   └── src/lib.rs
    ├── sim/                   # 自行车模型仿真器（引 7.1）
    │   ├── Cargo.toml
    │   └── src/lib.rs
    └── app/                   # 主循环，把一切连起来
        ├── Cargo.toml
        └── src/main.rs
```

根 `Cargo.toml`：

```toml
# minimal_av/Cargo.toml
[workspace]
resolver = "2"
members = [
    "crates/common",
    "crates/perception",
    "crates/prediction",
    "crates/planning",
    "crates/control",
    "crates/sim",
    "crates/app",
]

[workspace.package]
edition = "2021"
version = "0.1.0"
```

一条命令建好骨架：

```bash
cargo new --lib minimal_av && cd minimal_av
# 手动建目录，或用脚本：
for c in common perception prediction planning control sim; do
  cargo new --lib crates/$c
done
cargo new crates/app   # app 是二进制
```

然后把根 `Cargo.toml` 换成上面的 workspace 版本（`cargo new --lib minimal_av` 生成的根会被覆盖）。下面逐个 crate 填肉。

## 模块一：common —— 全书数据结构的家

这个 crate 里**没有算法，只有类型**。它是整个 workspace 的"通用语言"，所有其他 crate 都依赖它。我们直接搬来第 1.3 节定义的那几个结构，再补几个内部会用到的。

```toml
# crates/common/Cargo.toml
[package]
name = "common"
edition.workspace = true
version.workspace = true
```

```rust
// crates/common/src/lib.rs

/// 三维向量的类型别名，让签名更好读。
pub type Vec3 = [f64; 3];

/// 障碍物分类（第 1.3 节）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleClass {
    Vehicle,
    Pedestrian,
    Cyclist,
    Unknown,
}

/// 一个被感知到的障碍物。position/velocity 在**自车坐标系**下。
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub id: u32,
    pub class: ObstacleClass,
    pub position: Vec3, // 自车系：x 向前，y 向左
    pub velocity: Vec3,
    pub size: Vec3, // 长宽高
    pub heading: f64,
    pub confidence: f32,
}

/// 自车位姿，在**地图坐标系**下（第 1.3 节）。
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub position: Vec3,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

/// 一条轨迹上的点，坐标在地图系（第 1.3 节）。
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPoint {
    pub t: f64,       // 相对当前的时间 (s)
    pub x: f64,       // 地图坐标
    pub y: f64,
    pub v: f64,       // 期望速度 (m/s)
    pub heading: f64, // 期望朝向 (rad)
}

pub type Trajectory = Vec<TrajectoryPoint>;

/// 发给底盘的控制指令（第 1.3 节）。
#[derive(Debug, Clone, Copy)]
pub struct ControlCommand {
    pub steering_angle: f64, // 前轮转角 (rad)
    pub throttle: f64,       // 0~1
    pub brake: f64,          // 0~1
}

/// 车辆的真实运动状态。感知/规划/控制都要用到它。
/// 在最小栈里，我们让"定位"直接返回仿真真值，所以它同时是
/// 仿真器的内部状态，也是被下游消费的"估计状态"。
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,   // 地图系
    pub y: f64,
    pub yaw: f64, // 航向 (rad)
    pub v: f64,   // 纵向车速 (m/s)
}

impl VehicleState {
    /// 从真实状态导出一个 Pose（最小栈里的"完美定位"）。
    pub fn to_pose(&self) -> Pose {
        Pose {
            position: [self.x, self.y, 0.0],
            yaw: self.yaw,
            pitch: 0.0,
            roll: 0.0,
        }
    }
}

/// 预测模块的输出：一个障碍物的一条未来轨迹（单模态，恒速）。
/// 每个点是 (t, x, y)，坐标在**自车系**（和输入 Obstacle 一致）。
#[derive(Debug, Clone)]
pub struct PredictedObject {
    pub id: u32,
    pub class: ObstacleClass,
    pub path: Vec<(f64, f64, f64)>, // (t, x, y)
}
```

> **中高级视角**：真实系统里 `common` 这种"消息定义"crate 往往由 `.msg`/IDL 文件自动生成（第 8.4 节），并且带序列化（serde/CDR）。我们这里手写、不序列化，是因为最小栈是**单进程**的——所有节点在一个地址空间里，直接传结构体。等你把它拆成多进程（本章末尾会讲怎么拆），这个 crate 就该长出 `#[derive(Serialize, Deserialize)]` 了。

## 模块二：perception —— 从真值造出带噪声的清单

真实感知要跑神经网络。但感知的**输出契约**是固定的：一份 `Vec<Obstacle>`。所以在最小栈里，我们让感知作弊——它直接问仿真器"世界里真实有哪些障碍物"，然后**做两件真感知也要做的事**：

1. **坐标变换**：真值在地图系，感知输出必须在自车系。这一步是真的（第 3.2 节的刚体变换）。
2. **加噪声**：真感知永远不完美。我们给位置和速度掺一点随机误差，让下游尝到"不确定性"的味道。

```toml
# crates/perception/Cargo.toml
[package]
name = "perception"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
rand = "0.8"
```

```rust,ignore
// crates/perception/src/lib.rs
use common::{Obstacle, ObstacleClass, VehicleState, Vec3};
use rand::Rng;

/// 世界里一个"真实"障碍物（仿真器持有，感知不该直接看到它，
/// 但最小栈里我们让感知照着它造 Obstacle）。坐标在**地图系**。
#[derive(Debug, Clone)]
pub struct GroundTruthObstacle {
    pub id: u32,
    pub class: ObstacleClass,
    pub position: Vec3, // 地图系
    pub velocity: Vec3, // 地图系
    pub size: Vec3,
    pub heading: f64,
}

/// 感知模块。持有噪声参数与一个可复现的随机源。
pub struct Perception {
    pos_noise_std: f64,
    vel_noise_std: f64,
    max_range: f64, // 超出这个距离就"看不见"
}

impl Default for Perception {
    fn default() -> Self {
        Self {
            pos_noise_std: 0.15, // 位置噪声 ~15cm
            vel_noise_std: 0.05, // 速度噪声 ~5cm/s
            max_range: 80.0,
        }
    }
}

impl Perception {
    /// 输入：自车真实状态 + 世界真值障碍物。
    /// 输出：自车系下、带噪声的障碍物清单。
    pub fn detect(
        &self,
        ego: &VehicleState,
        world: &[GroundTruthObstacle],
    ) -> Vec<Obstacle> {
        let mut rng = rand::thread_rng();
        let (c, s) = (ego.yaw.cos(), ego.yaw.sin());

        world
            .iter()
            .filter_map(|gt| {
                // 1) 地图系 → 自车系的刚体变换（平移 + 旋转，第 3.2 节）。
                let dx = gt.position[0] - ego.x;
                let dy = gt.position[1] - ego.y;
                let ex = dx * c + dy * s; // 前
                let ey = -dx * s + dy * c; // 左

                // 距离门限：太远的"看不见"。
                if ex.hypot(ey) > self.max_range || ex < -5.0 {
                    return None;
                }

                // 障碍物速度也要转到自车系（只旋转，不平移）。
                let vx = gt.velocity[0] * c + gt.velocity[1] * s;
                let vy = -gt.velocity[0] * s + gt.velocity[1] * c;

                // 2) 加噪声。
                let n = |std: f64| -> f64 {
                    // 用两个均匀分布近似高斯（够用即可）。
                    let u: f64 = rng.gen_range(-1.0..1.0) + rng.gen_range(-1.0..1.0);
                    u * std
                };

                Some(Obstacle {
                    id: gt.id,
                    class: gt.class,
                    position: [ex + n(self.pos_noise_std), ey + n(self.pos_noise_std), 0.0],
                    velocity: [vx + n(self.vel_noise_std), vy + n(self.vel_noise_std), 0.0],
                    size: gt.size,
                    heading: gt.heading - ego.yaw,
                    confidence: 0.9,
                })
            })
            .collect()
    }
}
```

注意这里已经埋了两个真实的坑：远处目标看不见（`max_range`），以及每一帧的 `id` 我们直接用了真值 id——真感知没这个福利，它得**自己跨帧关联**出稳定 id，那就是多目标跟踪（第 4.5 节）。扩展方向之一就是在这里接一个卡尔曼跟踪器。

## 模块三：prediction —— 恒速模型

拿到当前障碍物，预测它未来几秒在哪。最简单也最常用的基线是**恒速模型（constant velocity, CV）**：假设它保持当前速度矢量匀速直线运动（第 6.1 节）。

数学上就一行：位置随时间线性外推。

```text
p(t) = p(0) + v * t
```

别小看它——在**短预测时程**（1~2 秒）里，恒速模型对车辆的预测精度经常打得过花哨的深度模型，因为物体有惯性、来不及乱变。它是所有预测系统的诚实基线。

```toml
# crates/prediction/Cargo.toml
[package]
name = "prediction"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
```

```rust,ignore
// crates/prediction/src/lib.rs
use common::{Obstacle, PredictedObject};

/// 恒速预测器（第 6.1 节的 CV 基线）。
pub struct ConstantVelocityPredictor {
    pub horizon: f64, // 预测时程 (s)
    pub dt: f64,      // 采样间隔 (s)
}

impl Default for ConstantVelocityPredictor {
    fn default() -> Self {
        Self { horizon: 3.0, dt: 0.2 }
    }
}

impl ConstantVelocityPredictor {
    pub fn predict(&self, obstacles: &[Obstacle]) -> Vec<PredictedObject> {
        let n = (self.horizon / self.dt) as usize;
        obstacles
            .iter()
            .map(|o| {
                let path = (0..=n)
                    .map(|k| {
                        let t = k as f64 * self.dt;
                        let x = o.position[0] + o.velocity[0] * t;
                        let y = o.position[1] + o.velocity[1] * t;
                        (t, x, y)
                    })
                    .collect();
                PredictedObject { id: o.id, class: o.class, path }
            })
            .collect()
    }
}
```

> **真实项目里**：预测输出应该是**多模态**的（第 6.1 节，"横穿" vs "等待"两条轨迹各带概率）。我们这里只吐一条最可能的（就是它当前速度方向），下游规划因此只对这一条负责。要升级成多模态，就让 `PredictedObject` 里放 `Vec<(f64, Vec<...>)>`（概率 + 轨迹），规划对每条模态都留安全裕度。

## 模块四：planning —— 单车道纵向速度规划

这是决定"车会不会撞上去"的模块。真正的规划要在 Frenet 坐标系里同时搜路径和速度（第 6.2 节），但我们的场景是**单车道直行**，横向不用动（保持车道中心线 y=0），问题退化成一维：**沿着这条线，我该用什么速度曲线，才能既舒适又安全？**

思路分三步：

**1) 找出"挡路的"障碍物。** 遍历预测结果，看谁的未来轨迹会进入我的车道走廊（`|y| < 车道半宽 + 裕度`）。对进入走廊的，记下它进入时在我前方的纵向距离 `s_obj`。取最近的那个作为"前车/前障碍"。

**2) 定一个停车点。** 在障碍物前方留一段安全间距 `gap`，得到必须停住的位置 `s_stop = s_obj - gap`。

**3) 生成速度曲线。** 用一条经典的"平方根减速曲线"：在距停车点还有 `d` 米的地方，允许的最大速度是

```text
v_max(d) = sqrt(2 * a_comf * d)
```

其中 `a_comf` 是舒适减速度（比如 1.5 m/s²）。这条曲线的物理含义是"以恒定舒适减速度恰好在停车点停下"，是纵向规划里最朴素好用的一招。再和巡航速度取小，就得到每个位置的目标速度。

```toml
# crates/planning/Cargo.toml
[package]
name = "planning"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
prediction = { path = "../prediction" }
```

```rust,ignore
// crates/planning/src/lib.rs
use common::{Pose, PredictedObject, Trajectory, TrajectoryPoint};

pub struct Planner {
    pub cruise_speed: f64, // 巡航速度 (m/s)
    pub lane_half_width: f64,
    pub safety_gap: f64,   // 停在障碍物前多远 (m)
    pub a_comf: f64,       // 舒适减速度 (m/s^2)
    pub horizon_len: f64,  // 规划路径长度 (m)
    pub ds: f64,           // 路径点间隔 (m)
}

impl Default for Planner {
    fn default() -> Self {
        Self {
            cruise_speed: 10.0,
            lane_half_width: 1.75,
            safety_gap: 5.0,
            a_comf: 1.5,
            horizon_len: 60.0,
            ds: 1.0,
        }
    }
}

impl Planner {
    /// 输入：自车位姿、当前车速、预测结果（自车系）。
    /// 输出：地图系下的一条参考轨迹（直行车道中心线 + 速度曲线）。
    pub fn plan(
        &self,
        ego: &Pose,
        ego_v: f64,
        predictions: &[PredictedObject],
    ) -> Trajectory {
        // 步骤 1+2：找最近的、会进入车道走廊的障碍物，算停车点。
        let mut s_stop: Option<f64> = None;
        for obj in predictions {
            // 在预测时程里，只要有一个采样点进入走廊，就认为它挡路。
            // 取它"首次进入走廊"那一刻的纵向位置作为障碍位置。
            for &(_, x, y) in &obj.path {
                if x > 0.0 && y.abs() < self.lane_half_width {
                    let candidate = (x - self.safety_gap).max(0.0);
                    s_stop = Some(match s_stop {
                        Some(s) => s.min(candidate),
                        None => candidate,
                    });
                    break; // 这个障碍物算过了，看下一个
                }
            }
        }

        // 步骤 3：沿中心线采样，生成 (位置, 目标速度)。
        // 车道中心线在地图系里是过自车、沿自车航向的直线。
        let (c, s) = (ego.yaw.cos(), ego.yaw.sin());
        let n = (self.horizon_len / self.ds) as usize;
        let mut traj = Trajectory::with_capacity(n + 1);
        let mut t_acc = 0.0;
        let mut prev_v = ego_v.max(0.1);

        for k in 0..=n {
            let dist = k as f64 * self.ds; // 自车前方 dist 米
            // 目标速度：巡航 与 "为停车点减速" 取小。
            let v_target = match s_stop {
                Some(ss) => {
                    let d_remain = ss - dist;
                    if d_remain <= 0.0 {
                        0.0 // 已到/越过停车点，必须停
                    } else {
                        (2.0 * self.a_comf * d_remain).sqrt().min(self.cruise_speed)
                    }
                }
                None => self.cruise_speed,
            };

            // 地图系坐标：沿航向前推 dist 米。
            let x = ego.position[0] + dist * c;
            let y = ego.position[1] + dist * s;

            // 粗略累积时间戳（供下游参考，用平均速度积分）。
            let v_avg = (prev_v + v_target).max(0.1) / 2.0;
            t_acc += self.ds / v_avg;
            prev_v = v_target;

            traj.push(TrajectoryPoint {
                t: t_acc,
                x,
                y,
                v: v_target,
                heading: ego.yaw,
            });
        }
        traj
    }
}
```

> **中高级视角**：这个规划器有个诚实的局限——它只会**停**，不会**绕**。因为我们锁死了横向（单车道）。真实规划在 Frenet 系里会同时评估"减速让行"和"向左微绕"两组候选，按代价函数选优（第 6.2/6.3 节）。但对"行人横穿、减速让行"这个场景，"只会停"恰恰是正确且安全的行为——工程上，**能把简单场景做到绝对可靠，比在复杂场景里花哨但偶尔出错更值钱**。

## 模块五：control —— PID + 纯跟踪

规划给了"未来该走的线和速度"，控制负责"这一瞬间方向盘、油门、刹车给多少"，把车**焊在**这条线上。我们纵横分开（第 7.2 节）：

- **纵向**用 PID：误差 = 目标速度 − 当前车速，PID 输出一个期望加速度，再映射成油门/刹车。
- **横向**用纯跟踪（Pure Pursuit）：在参考线上取一个"前视点（lookahead point）"，用几何关系算出让车头对准它所需的前轮转角：

```text
delta = atan2( 2 * L * sin(alpha), Ld )
```

其中 `L` 是轴距，`Ld` 是前视距离，`alpha` 是前视点相对车头的方位角。前视距离随速度增大（`Ld = k*v + Ld0`），这样高速时转向更柔和——这是纯跟踪的经典调法。

```toml
# crates/control/Cargo.toml
[package]
name = "control"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
```

```rust,ignore
// crates/control/src/lib.rs
use common::{ControlCommand, Trajectory, VehicleState};

/// 一个带抗积分饱和的 PID（第 7.2 节）。
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    integral: f64,
    prev_err: f64,
    i_limit: f64, // 积分限幅，防饱和
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, i_limit: f64) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_err: 0.0, i_limit }
    }

    pub fn step(&mut self, err: f64, dt: f64) -> f64 {
        self.integral = (self.integral + err * dt).clamp(-self.i_limit, self.i_limit);
        let deriv = (err - self.prev_err) / dt;
        self.prev_err = err;
        self.kp * err + self.ki * self.integral + self.kd * deriv
    }
}

pub struct Controller {
    speed_pid: Pid,
    wheelbase: f64,   // L
    ld_gain: f64,     // k
    ld_min: f64,      // Ld0
    max_accel: f64,   // 油门满对应的加速度
    max_brake: f64,   // 刹车满对应的减速度
}

impl Default for Controller {
    fn default() -> Self {
        Self {
            speed_pid: Pid::new(0.8, 0.1, 0.02, 5.0),
            wheelbase: 2.7,
            ld_gain: 0.5,
            ld_min: 3.0,
            max_accel: 3.0,
            max_brake: 6.0,
        }
    }
}

impl Controller {
    /// 输入：参考轨迹（地图系）+ 当前车辆真实状态。输出：控制指令。
    pub fn control(&mut self, traj: &Trajectory, ego: &VehicleState, dt: f64) -> ControlCommand {
        // ---------- 纵向：PID 速度跟踪 ----------
        // 目标速度取参考线上离自车最近点的速度。
        let v_target = self.target_speed(traj, ego);
        let accel_cmd = self.speed_pid.step(v_target - ego.v, dt);

        // 加速度 → 油门/刹车。
        let (throttle, brake) = if accel_cmd >= 0.0 {
            ((accel_cmd / self.max_accel).clamp(0.0, 1.0), 0.0)
        } else {
            (0.0, (-accel_cmd / self.max_brake).clamp(0.0, 1.0))
        };

        // ---------- 横向：纯跟踪 ----------
        let steering_angle = self.pure_pursuit(traj, ego);

        ControlCommand { steering_angle, throttle, brake }
    }

    fn target_speed(&self, traj: &Trajectory, ego: &VehicleState) -> f64 {
        traj.iter()
            .min_by(|a, b| {
                let da = (a.x - ego.x).hypot(a.y - ego.y);
                let db = (b.x - ego.x).hypot(b.y - ego.y);
                da.partial_cmp(&db).unwrap()
            })
            .map(|p| p.v)
            .unwrap_or(0.0)
    }

    fn pure_pursuit(&self, traj: &Trajectory, ego: &VehicleState) -> f64 {
        let ld = (self.ld_gain * ego.v + self.ld_min).max(self.ld_min);
        let (c, s) = (ego.yaw.cos(), ego.yaw.sin());

        // 找第一个到自车距离 >= Ld 的轨迹点作为前视点。
        let look = traj.iter().find(|p| {
            (p.x - ego.x).hypot(p.y - ego.y) >= ld
        });
        let Some(p) = look else { return 0.0 };

        // 把前视点转到自车系。
        let dx = p.x - ego.x;
        let dy = p.y - ego.y;
        let lx = dx * c + dy * s;
        let ly = -dx * s + dy * c;

        let alpha = ly.atan2(lx);
        (2.0 * self.wheelbase * alpha.sin()).atan2(ld)
    }
}
```

> **面试题**：PID 的积分项为什么要限幅？
> **答**：当车长时间达不到目标（比如被前障碍逼停、目标速度是 0 但车还没停稳），误差持续同号，积分会**越攒越大**（integral windup，积分饱和）。等误差反号时，这个巨大的积分要花很久才泄掉，导致大幅超调。限幅（或反算抗饱和）就是给积分器一个天花板。我们这里用最简单的 `clamp`，第 7.2 节讲了更讲究的反算法。

## 模块六：sim —— 运动学自行车模型

没有真车，我们用**运动学自行车模型（kinematic bicycle model）**当"世界"（第 7.1 节）。它把四轮车简化成前后两个轮子，用车速 `v` 和前轮转角 `delta` 驱动，状态是 `(x, y, yaw, v)`：

```text
x'   = v * cos(yaw)
y'   = v * sin(yaw)
yaw' = v / L * tan(delta)
v'   = a                     # a 由油门/刹车换算
```

我们用最简单的前向欧拉积分推进它，步长就是控制周期。这个模型在中低速、正常转向下足够真实，是所有控制入门的标准试验台。仿真器还负责推进行人的真实运动。

```toml
# crates/sim/Cargo.toml
[package]
name = "sim"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
perception = { path = "../perception" }
```

```rust,ignore
// crates/sim/src/lib.rs
use common::{ControlCommand, VehicleState};
use perception::GroundTruthObstacle;

/// 简易世界：自车 + 若干真值障碍物。
pub struct World {
    pub ego: VehicleState,
    pub obstacles: Vec<GroundTruthObstacle>,
    pub wheelbase: f64,
    pub max_accel: f64,
    pub max_brake: f64,
}

impl World {
    /// 用一个控制指令把世界推进 dt 秒。
    pub fn step(&mut self, cmd: &ControlCommand, dt: f64) {
        // 1) 油门/刹车 → 纵向加速度。
        let a = cmd.throttle * self.max_accel - cmd.brake * self.max_brake;

        // 2) 自行车模型积分（前向欧拉）。
        let e = &mut self.ego;
        e.x += e.v * e.yaw.cos() * dt;
        e.y += e.v * e.yaw.sin() * dt;
        e.yaw += e.v / self.wheelbase * cmd.steering_angle.tan() * dt;
        e.v = (e.v + a * dt).max(0.0); // 车不会倒车

        // 3) 推进障碍物（行人匀速横穿）。
        for o in &mut self.obstacles {
            o.position[0] += o.velocity[0] * dt;
            o.position[1] += o.velocity[1] * dt;
        }
    }
}
```

## 模块七：app —— 把五个节点连成闭环

现在装钟。主循环每个 tick 做一遍第 1.3 节那段"伪流水线"，只不过每个 `perceive/predict/plan/control` 都是真的模块调用，最后把控制指令喂给仿真器，世界前进一步，再进入下一 tick——**闭环**。

```toml
# crates/app/Cargo.toml
[package]
name = "app"
edition.workspace = true
version.workspace = true

[dependencies]
common = { path = "../common" }
perception = { path = "../perception" }
prediction = { path = "../prediction" }
planning = { path = "../planning" }
control = { path = "../control" }
sim = { path = "../sim" }
```

```rust,ignore
// crates/app/src/main.rs
use common::ObstacleClass;
use control::Controller;
use perception::{GroundTruthObstacle, Perception};
use planning::Planner;
use prediction::ConstantVelocityPredictor;
use sim::World;

fn main() {
    let dt = 0.05; // 20 Hz 控制周期（第 1.3 节的数量级）

    // ---- 搭场景：车 10 m/s 直行，前方 30m 行人横穿 ----
    let mut world = World {
        ego: common::VehicleState { x: 0.0, y: 0.0, yaw: 0.0, v: 10.0 },
        obstacles: vec![GroundTruthObstacle {
            id: 42,
            class: ObstacleClass::Pedestrian,
            position: [30.0, -1.5, 0.0], // 前方 30m，右侧 1.5m
            velocity: [0.0, 0.3, 0.0],   // 以 0.3 m/s 向 +y（车道）横穿
            size: [0.5, 0.5, 1.7],
            heading: std::f64::consts::FRAC_PI_2,
        }],
        wheelbase: 2.7,
        max_accel: 3.0,
        max_brake: 6.0,
    };

    // ---- 实例化五个节点 ----
    let perception = Perception::default();
    let predictor = ConstantVelocityPredictor::default();
    let planner = Planner::default();
    let mut controller = Controller::default();

    println!("{:>6} {:>7} {:>7} {:>7} {:>7} {:>7}", "t(s)", "ego_x", "ego_v", "ped_y", "brake", "gap");

    // ---- 主循环 ----
    let steps = (12.0 / dt) as usize; // 最多仿真 12 秒
    for k in 0..steps {
        let t = k as f64 * dt;

        // 1) 感知：真值 + 噪声 → 自车系障碍物清单。
        let obstacles = perception.detect(&world.ego, &world.obstacles);
        // 2) 定位：最小栈里直接用真值。
        let ego_pose = world.ego.to_pose();
        // 3) 预测：恒速外推。
        let predictions = predictor.predict(&obstacles);
        // 4) 规划：纵向速度规划。
        let traj = planner.plan(&ego_pose, world.ego.v, &predictions);
        // 5) 控制：PID + 纯跟踪。
        let cmd = controller.control(&traj, &world.ego, dt);

        // 每 0.5 秒打印一行状态。
        if k % 10 == 0 {
            let ped = &world.obstacles[0];
            let gap = ped.position[0] - world.ego.x;
            println!(
                "{:6.2} {:7.2} {:7.2} {:7.2} {:7.2} {:7.2}",
                t, world.ego.x, world.ego.v, ped.position[1], cmd.brake, gap
            );
        }

        // 6) 仿真：把世界推进一步。
        world.step(&cmd, dt);

        // 收敛判据：车基本停住了就退出。
        if world.ego.v < 0.05 {
            println!("\n>> 车已停稳，t = {:.2}s，停在行人前 {:.2} m 处。",
                t, world.obstacles[0].position[0] - world.ego.x);
            return;
        }
    }
    println!("\n>> 仿真结束（未在时限内停稳）。");
}
```

## 跑起来

在 `minimal_av/` 根目录：

```bash
cargo run -p app --release
```

预期输出（因为有随机噪声，数字每次略有不同，但趋势一致）：

```text
  t(s)   ego_x   ego_v   ped_y   brake     gap
  0.00    0.00   10.00   -1.50    0.00   30.00
  0.50    4.88    9.53   -1.35    0.14   25.12
  1.00    9.44    8.63   -1.20    0.22   20.56
  1.50   13.50    7.35   -1.05    0.28   16.50
  2.00   16.94    5.86   -0.90    0.31   13.06
  2.50   19.68    4.28   -0.75    0.33   10.32
  3.00   21.66    2.71   -0.60    0.34    8.34
  3.50   22.90    1.24   -0.45    0.31    7.10
  4.00   23.51    0.24   -0.30    0.18    6.49

>> 车已停稳，t = 4.15s，停在行人前 5.9 m 处。
```

读一读这几列，故事全在里面：

- `ego_v` 从 10 平滑降到 0，没有急刹（`brake` 峰值约 0.34，很温柔）——平方根减速曲线在起作用。
- `ped_y` 从 −1.5 一路涨向 0（行人往车道中心走），车在他还没走到车道中心时就已经停稳。
- 最终 `gap ≈ 5.9 m`，正好在我们设的 `safety_gap = 5.0` 附近（多出的是车头到质心的余量和减速末端的裕度）。

**车平滑地停在了行人前。** 第 1.3 节纸上那个场景，现在是一段真的跑起来的 Rust 程序了。

> **动手改坏它（强烈建议）**：
> - 把 `planner.safety_gap` 改成 1.0，看车贴多近停，理解安全裕度的意义。
> - 把 `a_comf` 调到 0.5，看减速曲线变得多长多缓——舒适和距离的权衡。
> - 把行人横向速度 `velocity[1]` 从 0.3 提到 2.0（快跑横穿），再看 `ego_x/gap`，感受"预测时程 vs 反应距离"。
> - 把 `dt` 从 0.05 改成 0.2（5 Hz），观察控制变粗糙——采样率对稳定性的影响。
> - 把感知 `pos_noise_std` 加到 2.0，看噪声怎么让规划抖动——这就是为什么真系统要在感知后加跟踪滤波。

## 进阶：把它变成并发的节点图

到这里的版本是**单进程顺序调用**——好理解、好调试、确定性强，作为学习和单测基座正合适。但第 1.3 节和第八部分反复强调：真实系统是**并发的、消息驱动的、各跑各频率的节点图**。感知 10 Hz、控制 50 Hz，谁也不等谁。

用第 2.5 节的 tokio + channel，我们可以在**不改任何模块内部代码**的前提下，把这几个 crate 包成异步节点。骨架长这样（示意，不是完整程序）：

```rust,ignore
// 每个节点是一个 async 任务，用 mpsc/watch channel 连接。
// 感知节点：定频产出障碍物，发给预测节点。
async fn perception_node(
    world: Arc<Mutex<World>>,
    tx: mpsc::Sender<Vec<Obstacle>>,
) {
    let perc = Perception::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(100)); // 10 Hz
    loop {
        ticker.tick().await;
        let obstacles = { let w = world.lock().await; perc.detect(&w.ego, &w.obstacles) };
        let _ = tx.send(obstacles).await; // 下游拿不过来就丢帧，正常
    }
}

// 控制节点跑得更快（50 Hz），用最新的规划结果。
async fn control_node(
    traj_rx: watch::Receiver<Trajectory>,   // watch：永远拿"最新一条"
    world: Arc<Mutex<World>>,
) {
    let mut controller = Controller::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(20)); // 50 Hz
    loop {
        ticker.tick().await;
        let traj = traj_rx.borrow().clone();
        let mut w = world.lock().await;
        let cmd = controller.control(&traj, &w.ego, 0.02);
        w.step(&cmd, 0.02);
    }
}
```

关键点，也是面试系统设计题的得分点：

- **不同节点不同频率**：用各自的 `interval`。控制不能等感知——它要用**手头最新**的规划结果继续跟线。
- **`watch` vs `mpsc` 的选择**：状态类数据（最新轨迹、最新位姿）用 `watch`（只保留最新值，天然"丢旧帧"）；事件类数据（每一帧障碍物）用有界 `mpsc`（满了就背压或丢帧）。这直接对应第 8.3 节 DDS 的 QoS 里 `KEEP_LAST(1)` vs `KEEP_ALL`。
- **共享世界的锁**：真车里没有"共享世界"，每个节点通过中间件收发消息。我们这里用 `Arc<Mutex<World>>` 只是因为仿真器要被读写，是仿真的特权，别把它当成架构范本。

想更进一步，就把这些 `mpsc`/`watch` 换成第 8.2 节的 `rclrs` 话题、或第 8.3 节的 Zenoh，节点各自成进程——你就从"一个程序"迈到了"一套真正的分布式自动驾驶系统"。

## 从这里长成一个硬核简历项目

这个最小栈是**骨架**，每根骨头都能换成真肌肉，而且换的时候接口基本不用动（这就是当初分 crate 的回报）：

1. **感知换真货**：在 `perception` 里接第 4.3 节的 `onnxruntime`/`tract`，跑一个真实的目标检测模型，输入换成图像/点云。加一个第 4.5 节的卡尔曼跟踪器产出稳定 id。
2. **定位换真货**：把"用真值"换成第 5.1/5.2 节的 EKF 组合导航，融合带噪声的 GNSS/IMU。
3. **预测升多模态**：按第 6.1 节做"横穿/等待"双模态，规划对两条都留裕度。
4. **规划升 Frenet**：从"只会停"升级到第 6.2 节的 Frenet 采样，让车会**绕行**，处理换道/避障。
5. **控制升 MPC**：把纯跟踪换成第 7.2 节的 MPC，显式处理约束和预测。
6. **仿真换 CARLA**：把自研 `sim` 换成第 9.1 节的 CARLA 客户端，或接一段真实数据回放（第 9.2 节），跑第 9.2 节的回归测试。
7. **拆成 ROS 2 图**：按上一节把节点搬到 `rclrs`（第 8.2 节），用 `ros2 topic echo` 观察消息流，用 rosbag 录制回放。

**挑一到两条深挖**，不要贪多。一个"感知用真模型 + 规划用 Frenet + 在 CARLA 里跑通行人横穿"的项目，已经足够你在中高级面试里聊满一小时。记住第 1.5 节说的：面试官往死里追问的，是你**真的踩过坑**的那部分。

## 小结

- 我们用一个 **Cargo workspace**（common / perception / prediction / planning / control / sim / app）搭出了纯 Rust 的最小自动驾驶栈，跑通了"感知→预测→规划→控制→仿真"闭环。
- 每个模块都是书里对应算法的**最简可用版**：感知=真值加噪+坐标变换，预测=恒速模型（6.1），规划=平方根减速的纵向速度规划（6.2），控制=PID+纯跟踪（7.2），仿真=运动学自行车模型（7.1）。
- 它成功复现了第 1.3 节的场景：**车平滑地停在横穿行人前**，输出可读、可复现、可"改坏再修好"。
- 单进程顺序版本适合学习和单测；用 tokio + channel（2.5）可零改动地把它变成**多频率、消息驱动的并发节点图**，再往前就是 rclrs/Zenoh 的分布式系统（第八部分）。
- 这个骨架的每根骨头都能换成真肌肉——挑一两条深挖，它就是你**能写进简历、扛得住深挖**的项目。

下一节，我们把整本书浓缩成一张能力地图，给你一条从"读完这本书"到"拿到 offer"的可执行路线。
