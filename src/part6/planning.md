# 6.2 路径与轨迹规划

如果说预测是"猜别人要干嘛"，那规划（planning）就是"决定我自己要怎么走"。这是规划岗（planning engineer）的主战场，也是整本书里数学密度最高的一章。我不打算绕过任何硬骨头——A\*、五次多项式、Frenet 坐标系，该推的公式推、该给的代码给。读完这一章，你手上会有几个能真的算出轨迹的 Rust 组件，脑子里会有一张"经典规划方法全景图"。深呼吸，我们开始。

## 先把"规划"这个词拆清楚：三层

新人最容易混的一件事：**"规划"不是一个算法，是一整层，内部还分三级**。工业界几乎所有栈都是这个分层：

1. **路由 / 全局路径（routing）**：在**道路网络**（road graph）这个尺度上，从起点到终点选一条路线——"走三环还是走高架"。这一层跑在整张城市地图上，秒级甚至分钟级更新一次，本质是图搜索（Dijkstra/A\*）。你手机导航干的就是这个。
2. **行为决策（behavior）**：在**当前这段路 + 周围交通**的尺度上，做**离散决策**——"跟车还是超车""让行还是通过""现在变道还是再等等"。这是下一章 6.3 的主题，输出的是约束和意图，不是具体轨迹。
3. **运动规划 / 轨迹生成（motion planning）**：在**未来几秒、几十米**的尺度上，把行为决策的意图落成一条**平滑、安全、动力学可行**的具体轨迹。这是本章的核心，几十毫秒出一次解。

这三层是**逐级细化**的关系：路由说"往前走然后右转"，行为说"右转前要先减速让掉那个行人、并到右道"，运动规划说"这是未来 5 秒每 0.1 秒你该在的精确 (x, y, v, heading)"。本章聚焦第 1 和第 3 层（第 2 层单独成章）。

## 路径 vs 轨迹：差一个"时间"，差很多

这是面试必问、新人必错的概念。

- **路径（path）**：一条几何曲线，只有空间信息——`(x, y)` 的序列，或者参数曲线 `(x(s), y(s))`。它回答"走哪条线"，**不回答"什么时候走到哪、走多快"**。
- **轨迹（trajectory）**：路径 + 时间维度——`(x(t), y(t))` 或者复用 1.3 节那个带 `t` 和 `v` 的 `TrajectoryPoint`。它回答"什么时刻在什么位置、多快"。

为什么必须区分？因为**同一条路径，配不同的速度剖面（velocity profile），是完全不同的驾驶行为**。一条绕过前车的路径，你快速通过还是慢慢挪，安全性天差地别。控制模块（第七部分）要的是**轨迹**——它每个瞬间都得知道"此刻我该在哪、该多快"，才能算方向盘和油门。所以规划的最终产物一定是带时间的**轨迹**。

工业界有两大流派处理这个问题：

- **路径-速度分解（path-velocity decomposition）**：先规划一条路径（横向：往哪拐），再在这条路径上规划速度（纵向：多快走）。Apollo 的 EM Planner 就是这个思路。好处是把一个难的二维时空问题拆成两个一维问题，各个击破。
- **时空联合（spatio-temporal / lattice）**：直接在 `(s, l, t)` 或 `(s, t)` 空间里一把梭，同时决定走位和速度。理论上更优（有些场景路径和速度耦合，分开会丢最优解），但计算量大。

记住这个分野，后面 Frenet 和 lattice 会反复用到。

## 经典方法全景

我把规划方法分成四大家族，逐个给你原理 + 代码。它们不是互斥的，真实系统往往组合使用（比如 Frenet 采样 + QP 优化）。

### 家族一：图搜索（graph search）

**核心思想**：把连续空间**离散化成图**（格子、路网节点），然后在图上找最短路。

#### Dijkstra 与 A\*

Dijkstra 你可能听过——从起点向外一圈圈"涨水"，涨到终点为止，保证找到最短路。它的问题是**没有方向感**：终点在东边，它也往西边使劲搜，浪费。

**A\*** 是 Dijkstra 加了个"指南针"。它给每个节点一个评估函数：

$$
f(n) = g(n) + h(n)
$$

- `g(n)`：从起点到 `n` 的**实际**代价（已经走过的路）。
- `h(n)`：从 `n` 到终点的**估计**代价（还要走多远的猜测），叫**启发函数（heuristic）**。
- `f(n)`：这条路"总代价"的估计。A\* 每次优先扩展 `f` 最小的节点。

关键理论保证：只要 `h(n)` **不高估**真实剩余代价（这叫**可采纳性 admissible**），A\* 就一定能找到最优解。而 `h` 越接近真实剩余代价，搜索越快、越"直奔目标"。在栅格地图上常用的启发函数：

- **曼哈顿距离** `|dx| + |dy|`：只能上下左右走时用。
- **欧氏距离** `sqrt(dx² + dy²)`：能任意方向走时用，永不高估（直线最短）。
- **对角距离（Chebyshev/octile）**：能走八方向时用。

下面是**可运行**的栅格 A\*，用欧氏启发。注意 Rust 里用 `BinaryHeap` 做优先队列的经典技巧——因为它是**大顶堆**，我们要取最小 `f`，得把代价取反（`Reverse`）。

```rust
// Cargo.toml 无需额外依赖
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

/// 栅格坐标
type Cell = (i32, i32);

/// 优先队列里的条目：按 f 排序（f 小的优先）
#[derive(Copy, Clone, PartialEq)]
struct Node {
    f: f64,
    g: f64,
    cell: Cell,
}
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // 注意：反向比较，让 BinaryHeap（大顶堆）弹出最小 f
        other.f.partial_cmp(&self.f)
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

pub struct Grid {
    pub w: i32,
    pub h: i32,
    pub blocked: Vec<bool>, // 长度 w*h，true 表示障碍
}
impl Grid {
    fn idx(&self, c: Cell) -> usize {
        (c.1 * self.w + c.0) as usize
    }
    fn passable(&self, c: Cell) -> bool {
        c.0 >= 0 && c.0 < self.w && c.1 >= 0 && c.1 < self.h && !self.blocked[self.idx(c)]
    }
    /// 八邻域
    fn neighbors(&self, c: Cell) -> Vec<(Cell, f64)> {
        const DIRS: [(i32, i32); 8] = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ];
        DIRS.iter()
            .map(|&(dx, dy)| ((c.0 + dx, c.1 + dy), if dx != 0 && dy != 0 { 1.4142 } else { 1.0 }))
            .filter(|(nc, _)| self.passable(*nc))
            .collect()
    }
}

/// 欧氏距离启发函数（可采纳：直线是最短的，永不高估）
fn heuristic(a: Cell, b: Cell) -> f64 {
    (((a.0 - b.0).pow(2) + (a.1 - b.1).pow(2)) as f64).sqrt()
}

/// A* 搜索，返回从 start 到 goal 的路径（含两端），无解返回 None
pub fn astar(grid: &Grid, start: Cell, goal: Cell) -> Option<Vec<Cell>> {
    let mut open = BinaryHeap::new();
    let mut g_score: HashMap<Cell, f64> = HashMap::new();
    let mut came_from: HashMap<Cell, Cell> = HashMap::new();

    g_score.insert(start, 0.0);
    open.push(Node { f: heuristic(start, goal), g: 0.0, cell: start });

    while let Some(Node { g, cell, .. }) = open.pop() {
        if cell == goal {
            // 回溯重建路径
            let mut path = vec![cell];
            let mut cur = cell;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        // 惰性删除：如果这个节点已经有更优的 g，跳过（避免重复扩展）
        if g > *g_score.get(&cell).unwrap_or(&f64::INFINITY) {
            continue;
        }
        for (nc, cost) in grid.neighbors(cell) {
            let tentative = g + cost;
            if tentative < *g_score.get(&nc).unwrap_or(&f64::INFINITY) {
                came_from.insert(nc, cell);
                g_score.insert(nc, tentative);
                open.push(Node { f: tentative + heuristic(nc, goal), g: tentative, cell: nc });
            }
        }
    }
    None // 开集耗尽仍未到达
}

fn main() {
    // 10x10 网格，中间竖一堵墙，留个豁口
    let mut blocked = vec![false; 100];
    for y in 0..8 {
        blocked[(y * 10 + 5) as usize] = true; // x=5 这一列 y=0..8 是墙
    }
    let grid = Grid { w: 10, h: 10, blocked };
    match astar(&grid, (0, 0), (9, 9)) {
        Some(path) => {
            println!("找到路径，长度 {} 步：", path.len());
            for c in &path {
                print!("({},{}) ", c.0, c.1);
            }
            println!();
        }
        None => println!("无解"),
    }
}
```

跑起来你会看到 A\* 聪明地绕过墙、从下方豁口穿过去。**这就是全局路由那一层的核心引擎**——把城市路网当成图，节点是路口、边是道路段、边权是"这段路预计要开多久"，A\* 一跑就是导航路线。

> **面试题**：A\* 的启发函数如果**高估**了剩余代价会怎样？
> **答**：失去最优性保证——它可能过早地"扑向"目标而错过真正的最短路，返回一条次优路径。但换来的是搜得更快（扩展节点更少）。工程上有时故意用"加权 A\*"（`f = g + w·h, w>1`）牺牲一点最优性换速度，这在实时系统里是常见交易。

#### Hybrid A\*：给泊车用的"考虑车头朝向"的 A\*

普通栅格 A\* 有个硬伤：它产生的路径是"格子到格子"的折线，**忽略了车辆的运动学约束**——车不能原地平移、不能瞬间掉头，有最小转弯半径。让车按 A\* 的锯齿路径走，方向盘得抽风。

**Hybrid A\***（混合 A\*）是泊车、窄路掉头这类**低速、需精确操作**场景的王者。它的核心改动：

- **状态多一维朝向**：节点不再是 `(x, y)`，而是 `(x, y, θ)`——连续的位置和朝向。
- **扩展用运动学模型**：不是"跳到相邻格子"，而是**用自行车模型（bicycle model，第七部分详解）以几个离散的转向角向前滚一小段**，得到的下一个状态是**连续**的（落在格子内部，不吸附到格心）——"hybrid"就是指状态连续、但仍用栅格做剪枝去重。
- **两个启发函数取最大**：一个是"不考虑障碍、但考虑运动学"的 Reeds-Shepp 曲线长度（车实际能怎么走），一个是"考虑障碍、但不考虑运动学"的栅格 Dijkstra 距离。两者取 max，兼顾"车走得动"和"绕得开"。

Hybrid A\* 产出的路径**天生满足最小转弯半径**，方向盘打得动，泊车能一把入库或规整地揉库。完整实现较长（要引入自行车模型和 Reeds-Shepp 曲线），这里给你扩展节点的骨架，建立直觉：

```rust
/// 连续状态：位置 + 朝向
#[derive(Clone, Copy)]
struct SE2 { x: f64, y: f64, theta: f64 }

/// 用自行车模型前滚一步：以转向角 steer、步长 ds 前进
fn kinematic_step(s: SE2, steer: f64, ds: f64, wheelbase: f64) -> SE2 {
    // 自行车模型：航向变化 = 前进距离 / 转弯半径, 转弯半径 = L / tan(steer)
    let dtheta = ds * steer.tan() / wheelbase;
    SE2 {
        x: s.x + ds * s.theta.cos(),
        y: s.y + ds * s.theta.sin(),
        theta: s.theta + dtheta,
    }
}

/// 扩展：对几个离散转向角各前滚一步，得到连续的后继状态
fn expand(s: SE2, wheelbase: f64) -> Vec<SE2> {
    const STEERS: [f64; 5] = [-0.5, -0.25, 0.0, 0.25, 0.5]; // 几档方向盘
    const DS: f64 = 0.8; // 每步前进 0.8 m
    STEERS.iter().map(|&st| kinematic_step(s, st, DS, wheelbase)).collect()
    // 真实实现里：把 (x,y,theta) 量化到栅格做 visited 去重，
    // 代价里加"倒车惩罚""转向变化惩罚"，启发用 Reeds-Shepp + 栅格 Dijkstra 取 max
}
```

### 家族二：采样（sampling-based）

图搜索在高维空间会爆炸（状态维度一多，格子数指数增长）。**采样法**换个思路：**不系统地铺满整个空间，而是随机撒点、连成树，撞运气般地探到目标**。

- **RRT（Rapidly-exploring Random Tree，快速探索随机树）**：从起点长一棵树。每轮在空间里随机采一个点，找树上离它最近的节点，朝它伸一小步（如果不撞障碍），把新节点接上树。树会"贪婪地"向未探索区域扩张，很快铺满空间，直到碰到目标附近。优点是**天然处理高维和复杂约束**、实现简单、能快速找到**一条**可行解。缺点是那条解**又丑又不最优**（随机的锯齿路径），需要后处理平滑。

- **RRT\***：RRT 的最优化版本。多做两件事——新节点接入时**在邻域里选代价最小的父节点**（choose parent），接入后**尝试用新节点作为跳板给邻居重新布线**（rewire）降低它们的代价。随着采样数增加，RRT\* 的解**渐近收敛到最优**。代价是慢一些。

采样法在自动驾驶里不如 Frenet + 优化主流（车道结构化环境更适合后者），但在**非结构化场景**（开阔停车场、越野、狭窄工地）很有价值——那里没有车道线可依附，RRT 这种"不挑食"的方法反而好使。RRT 扩展一步的核心逻辑：

```rust
/// RRT 单步扩展的核心：随机采样 → 找最近 → 朝它伸一步
fn rrt_extend(tree: &mut Vec<(f64, f64)>, parent: &mut Vec<usize>,
              sample: (f64, f64), step: f64,
              collision_free: impl Fn((f64, f64), (f64, f64)) -> bool) {
    // 1. 找树上离 sample 最近的节点
    let (nearest_i, &near) = tree.iter().enumerate()
        .min_by(|(_, a), (_, b)| {
            let da = (a.0 - sample.0).powi(2) + (a.1 - sample.1).powi(2);
            let db = (b.0 - sample.0).powi(2) + (b.1 - sample.1).powi(2);
            da.partial_cmp(&db).unwrap()
        }).unwrap();
    // 2. 朝 sample 方向伸 step 距离
    let (dx, dy) = (sample.0 - near.0, sample.1 - near.1);
    let d = (dx * dx + dy * dy).sqrt().max(1e-9);
    let new = (near.0 + step * dx / d, near.1 + step * dy / d);
    // 3. 不撞障碍才接上树
    if collision_free(near, new) {
        tree.push(new);
        parent.push(nearest_i);
    }
}
```

### 家族三：曲线拟合（curve generation）

图搜索和采样给的是**离散折线**，不够光滑，车开着颠。曲线家族反过来：**用一个光滑的参数函数直接描述轨迹**，光滑性由数学结构保证。

- **多项式曲线**：用 `x(t)`、`y(t)` 各是一个时间的多项式。想控制到几阶导数连续（位置、速度、加速度、加加速度 jerk），就用几次多项式。这是本节的重头戏，下面详讲。
- **样条（spline）**：分段多项式，在接缝处保证连续性（如三次样条保证到二阶导连续）。适合把 A\* 的折线路点平滑成能开的曲线。
- **贝塞尔曲线（Bézier）**：用几个控制点定义，曲线一定落在控制点的凸包内——这个性质在**保证不越界**时很有用。工业界画车道、平滑路径常用。

#### 重头戏：五次多项式生成横向轨迹

这是规划工程师的**基本功中的基本功**，请务必吃透。场景：车要平滑地变道，或从当前横向位置回到车道中心。我们希望这个横向运动**又快又舒适**——舒适意味着加速度不能突变（不然乘客被甩），也就是要控制到**加速度连续**。

数学上，"位置、速度、加速度"三个量在**起点和终点**都要满足给定条件，一共 **6 个边界条件**。要精确满足 6 个约束，多项式就需要 **6 个系数**——正好是一个**五次多项式（quintic polynomial）**：

$$
l(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5
$$

这里 `l` 是横向偏移（后面 Frenet 里就是那个 `l`），`t` 是时间。它的一阶、二阶导（横向速度、加速度）：

$$
\begin{aligned}
\dot l(t) &= c_1 + 2c_2 t + 3c_3 t^2 + 4c_4 t^3 + 5c_5 t^4 \\
\ddot l(t) &= 2c_2 + 6c_3 t + 12c_4 t^2 + 20c_5 t^3
\end{aligned}
$$

**边界条件**：给定起点 `t=0` 的 `(l_0, \dot l_0, \ddot l_0)` 和终点 `t=T` 的 `(l_T, \dot l_T, \ddot l_T)`。

在 `t=0` 处，多项式的低阶系数**直接就是初始条件**（把 `t=0` 代进去，高次项全归零）：

$$
c_0 = l_0, \quad c_1 = \dot l_0, \quad c_2 = \frac{\ddot l_0}{2}
$$

漂亮，一半系数白送。剩下 `c_3, c_4, c_5` 由终点 3 个条件确定。把 `t=T` 代入 `l, \dot l, \ddot l` 三个式子，移项后得到一个 3×3 线性方程组：

$$
\begin{bmatrix}
T^3 & T^4 & T^5 \\
3T^2 & 4T^3 & 5T^4 \\
6T & 12T^2 & 20T^3
\end{bmatrix}
\begin{bmatrix} c_3 \\ c_4 \\ c_5 \end{bmatrix}
=
\begin{bmatrix}
l_T - (c_0 + c_1 T + c_2 T^2) \\
\dot l_T - (c_1 + 2c_2 T) \\
\ddot l_T - 2c_2
\end{bmatrix}
$$

解这个 3×3 就得到全部系数。这个矩阵有解析逆，但直接手解容易错，我们在代码里用克拉默法则（Cramer's rule）或直接写出解析解。下面是**可运行**实现：

```rust
// Cargo.toml 无需额外依赖

/// 五次多项式，存 6 个系数
#[derive(Debug, Clone, Copy)]
pub struct Quintic {
    c: [f64; 6],
}

impl Quintic {
    /// 由起点/终点的 (位置, 速度, 加速度) 和总时长 T 求解系数
    pub fn new(
        l0: f64, dl0: f64, ddl0: f64, // 起点
        lt: f64, dlt: f64, ddlt: f64, // 终点
        t: f64,
    ) -> Self {
        // 前三个系数直接来自起点条件
        let c0 = l0;
        let c1 = dl0;
        let c2 = ddl0 / 2.0;

        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        // 右端向量 b：终点条件减去低阶项的贡献
        let b0 = lt - (c0 + c1 * t + c2 * t2);
        let b1 = dlt - (c1 + 2.0 * c2 * t);
        let b2 = ddlt - 2.0 * c2;

        // 系数矩阵 A（就是上面推导那个 3x3），解 A [c3 c4 c5]^T = b
        // 这里用解析解（对该结构矩阵求逆的已知结果），数值稳定且快
        let c3 = (10.0 * b0 / t3) - (4.0 * b1 / t2) + (0.5 * b2 / t);
        let c4 = (-15.0 * b0 / t4) + (7.0 * b1 / t3) - (1.0 * b2 / t2);
        let c5 = (6.0 * b0 / t5) - (3.0 * b1 / t4) + (0.5 * b2 / t3);

        Quintic { c: [c0, c1, c2, c3, c4, c5] }
    }

    /// 位置
    pub fn pos(&self, t: f64) -> f64 {
        let c = &self.c;
        c[0] + c[1] * t + c[2] * t.powi(2) + c[3] * t.powi(3) + c[4] * t.powi(4) + c[5] * t.powi(5)
    }
    /// 速度（一阶导）
    pub fn vel(&self, t: f64) -> f64 {
        let c = &self.c;
        c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t.powi(2) + 4.0 * c[4] * t.powi(3) + 5.0 * c[5] * t.powi(4)
    }
    /// 加速度（二阶导）
    pub fn acc(&self, t: f64) -> f64 {
        let c = &self.c;
        2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t.powi(2) + 20.0 * c[5] * t.powi(3)
    }
    /// 加加速度 jerk（三阶导，衡量舒适度）
    pub fn jerk(&self, t: f64) -> f64 {
        let c = &self.c;
        6.0 * c[3] + 24.0 * c[4] * t + 60.0 * c[5] * t.powi(2)
    }
}

fn main() {
    // 场景：车当前偏离车道中心 l=3.5m（在左车道），速度和加速度都为 0（横向静止），
    // 目标 2 秒内平滑回到中心 l=0，且到达时横向速度、加速度都为 0（稳稳停在中心）
    let q = Quintic::new(3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    println!(" t     l       dl      ddl     jerk");
    let mut t = 0.0;
    while t <= 2.0 + 1e-9 {
        println!("{:.1}  {:6.3}  {:6.3}  {:6.3}  {:6.3}",
                 t, q.pos(t), q.vel(t), q.acc(t), q.jerk(t));
        t += 0.25;
    }
    // 验证：t=0 时 l=3.5, dl=0, ddl=0；t=2 时 l≈0, dl≈0, ddl≈0
}
```

跑出来你会看到 `l` 从 3.5 平滑地、S 形地过渡到 0，中间横向速度先增后减、加速度两头为零——**这就是一次舒适的变道横向运动**。把终点 `l_T` 改成另一条车道的中心，就是变道；把它设成 0，就是"回正到车道中心"。**五次多项式之所以是横向规划的默认选择，就因为它用最低的阶数精确满足了"两端位置/速度/加速度全指定"这 6 个舒适性约束。**

> **中高级视角**：为什么横向用**五次**，纵向（速度规划）却常用**四次**？因为纵向我们通常**不关心终点到达的精确位置**（你不会要求"5 秒后必须恰好在 s=50.0 米"），只关心终点的**速度和加速度**（想巡航到某个目标速度）。少一个位置约束就少一个边界条件，5 个约束对应 4 次多项式。这个"按你真正在乎的边界条件数选多项式阶数"的思维，比记住"横向五次纵向四次"这个结论重要得多。

### 家族四：优化（optimization）——把规划写成一道数学题

前面三家各有各的味道，但最能代表现代规划的，是把整件事**表述成一个带约束的优化问题**：

$$
\min_{\text{trajectory}} \; J(\text{trajectory}) \quad \text{s.t. 约束}
$$

**代价函数 `J`** 是各种目标的加权和，典型三块：

$$
J = w_{\text{smooth}} \underbrace{\int (\ddot l^2 + \dddot l^2)\,dt}_{\text{平滑/舒适}} + w_{\text{ref}} \underbrace{\int (l - l_{\text{ref}})^2 dt}_{\text{贴合参考线/效率}} + w_{\text{safe}} \underbrace{\sum \text{离障碍物的惩罚}}_{\text{安全}}
$$

- **平滑项**：惩罚大的加速度和 jerk——坐着舒服、车辆动力学吃得消。
- **参考项**：惩罚偏离参考线（车道中心）或目标速度——别没事乱拐、别磨蹭。
- **安全项**：离障碍物越近惩罚越大——别贴着别人开。

**约束**则是硬性的：不能撞（碰撞约束）、方向盘打得过来（曲率约束）、加速度别超过轮胎抓地极限（动力学约束）、别开出路面（边界约束）。

当代价函数是**二次型**、约束是**线性**时，这个优化就是一个**二次规划（Quadratic Programming, QP）**——凸优化里性质最好、求解最快的一类，有成熟的数值求解器（OSQP、qpOASES 等），毫秒级出解，这正是实时规划所需要的。Apollo 的路径优化、速度优化都归结成 QP。

工业界两条著名路线：

- **Lattice Planner（网格/采样式规划）**：不直接解优化，而是**撒一大把候选轨迹**（用不同终点条件生成一堆五次/四次多项式），对每条算代价 `J`，**选代价最低且无碰撞的那条**。本质是"采样 + 打分"，简单、鲁棒、易调、天然多模态。缺点是解的质量受限于你撒的候选密度。
- **EM Planner / 迭代优化式**：Apollo 的经典方案，用**路径-速度分解**，路径和速度各自迭代地做"动态规划找粗解 + 二次规划精修"（DP 提供一个凸的可行域，QP 在里面求光滑最优）。质量更高但更复杂。

下面给一个**极简 lattice planner** 的骨架，把"多项式生成 + 代价评估 + 选优"串起来，你能直接看懂前面所有零件怎么组装：

```rust,ignore
/// 一条候选轨迹及其代价
struct Candidate {
    lat: Quintic,       // 横向运动（复用上面的五次多项式）
    target_l: f64,      // 终点横向偏移
    cost: f64,
}

/// 极简 lattice：对一组候选终点横向位置各生成一条五次轨迹，打分选最优
fn lattice_plan(
    l0: f64, dl0: f64, ddl0: f64, // 当前横向状态
    obstacles_l: &[f64],          // 障碍物在参考线横向的占据位置（简化）
    horizon: f64,
) -> Option<Quintic> {
    // 候选终点：车道中心及左右微调，覆盖"回中心/微避让"等模态
    let target_ls = [-1.0, -0.5, 0.0, 0.5, 1.0];
    let mut best: Option<Candidate> = None;

    for &lt in &target_ls {
        // 终点都要求横向速度、加速度为 0（稳定收敛）
        let q = Quintic::new(l0, dl0, ddl0, lt, 0.0, 0.0, horizon);

        // 采样若干时刻，累加代价
        let mut cost = 0.0;
        let mut collision = false;
        let mut t = 0.0;
        while t <= horizon + 1e-9 {
            let l = q.pos(t);
            // 平滑代价：加速度平方
            cost += q.acc(t).powi(2) * 0.1;
            // 参考代价：偏离车道中心 (l=0) 的惩罚
            cost += l.powi(2) * 1.0;
            // 安全：离任何障碍横向太近就判碰撞
            for &ol in obstacles_l {
                if (l - ol).abs() < 1.0 {
                    collision = true;
                }
            }
            t += 0.2;
        }
        if collision {
            continue; // 硬约束：撞了直接丢弃
        }
        if best.as_ref().map_or(true, |b| cost < b.cost) {
            best = Some(Candidate { lat: q, target_l: lt, cost });
        }
    }
    best.map(|b| b.lat)
}
```

这就是一个**能跑通的最小轨迹规划器**：生成一批候选、算代价、避碰、选优。工业级的无非是把它做厚——候选采样更密、代价项更全（加纵向、加时间维）、约束更严（动力学）、求解从"暴力选优"换成 QP。但**骨架就是这个骨架**。

## Frenet 坐标系：现代规划的地基

现在讲这一章、乃至整个规划领域**最重要**的一个思想。前面 lattice 里我一直用 `l`（横向偏移）而不是 `y`，A\* 里用 `s`（弧长），这不是随手写的——它们来自 **Frenet 坐标系（Frenet frame）**，也叫 **SL 坐标系**。

### 问题出在哪

在笛卡尔坐标 `(x, y)` 里描述"沿车道行驶"极其别扭。车道是弯的，"车道中心"是一条曲线，"离中心多远""沿车道走了多远"这些**规划真正关心的量**，在 `(x, y)` 里都得靠复杂的几何计算才能得到。弯道上"保持车道中心"意味着 `x` 和 `y` 同时按某种耦合方式变化——恶心。

### Frenet 的解法：换一个"跟着路走"的坐标系

Frenet 坐标系**沿着参考线（reference line，通常是车道中心线）建立**，用两个量描述任意位置：

- **`s`（纵向，longitudinal）**：沿参考线走过的**弧长**——"我沿着这条路开了多远"。
- **`l`（横向，lateral，也写作 `d`）**：垂直于参考线的**横向偏移**——"我偏离中心线多远"，左正右负（或反之，约定即可）。

这一换，奇迹发生了：

- **"沿车道行驶"变成 `l ≈ 0` 且 `s` 增大**——无论路多弯，在 Frenet 里都是"横向不动、纵向前进"，跟直路一模一样。
- **纵向和横向解耦了**：`s` 管"快慢"（速度规划、跟车、红灯停车），`l` 管"走位"（变道、避让）。前面"路径-速度分解"能成立，正是因为 Frenet 把二维耦合问题拆成了两个独立的一维问题。
- **五次多项式直接用在 `l(t)` 上**——我们前面那个"从 3.5 平滑回到 0"的例子，就是 Frenet 下的横向规划。障碍物、目标车道、避让边界，全都投影成 `(s, l)`，规划在这个"拉直了的路"上进行，算完再转回 `(x, y)` 交给控制。

**这就是为什么 Frenet 是现代规划的地基**：它把"在弯曲车道上规划"这个难题，变成了"在一条直路上规划"这个简单题。

### 坐标转换：Cartesian ↔ Frenet

核心是围绕参考线做投影。给定参考线（一串带累积弧长的点），把世界坐标点 `(x, y)` 转成 `(s, l)`：

1. 在参考线上找**离 `(x, y)` 最近的点**，它的弧长就是 `s`。
2. `(x, y)` 到那个最近点的距离就是 `|l|`，符号由"在参考线左侧还是右侧"决定（用叉积判断）。

反过来 `(s, l) → (x, y)`：在参考线上找到弧长 `s` 的点和该处的切向/法向，沿法向偏移 `l` 即可。下面是**可运行**实现（参考线用点列 + 累积弧长表示，复用上一章 `Centerline` 的思路）：

```rust
// Cargo.toml 无需额外依赖

pub struct ReferenceLine {
    pts: Vec<(f64, f64)>, // 参考线采样点（车道中心线）
    s: Vec<f64>,          // 累积弧长
}

impl ReferenceLine {
    pub fn new(pts: Vec<(f64, f64)>) -> Self {
        let mut s = vec![0.0];
        for i in 1..pts.len() {
            let d = ((pts[i].0 - pts[i - 1].0).powi(2)
                   + (pts[i].1 - pts[i - 1].1).powi(2)).sqrt();
            s.push(s[i - 1] + d);
        }
        ReferenceLine { pts, s }
    }

    /// 某个采样点处的单位切向量（用相邻点差分近似）
    fn tangent_at(&self, i: usize) -> (f64, f64) {
        let j = if i + 1 < self.pts.len() { i + 1 } else { i };
        let k = if i > 0 { i - 1 } else { i };
        let (dx, dy) = (self.pts[j].0 - self.pts[k].0, self.pts[j].1 - self.pts[k].1);
        let n = (dx * dx + dy * dy).sqrt().max(1e-9);
        (dx / n, dy / n)
    }

    /// Cartesian → Frenet: (x, y) → (s, l)
    pub fn to_frenet(&self, x: f64, y: f64) -> (f64, f64) {
        // 1. 找最近采样点
        let mut best_i = 0;
        let mut best_d2 = f64::INFINITY;
        for i in 0..self.pts.len() {
            let d2 = (self.pts[i].0 - x).powi(2) + (self.pts[i].1 - y).powi(2);
            if d2 < best_d2 {
                best_d2 = d2;
                best_i = i;
            }
        }
        let s = self.s[best_i];
        // 2. 横向偏移大小与符号
        let (tx, ty) = self.tangent_at(best_i);
        let (dx, dy) = (x - self.pts[best_i].0, y - self.pts[best_i].1);
        // 叉积 tangent × offset 的 z 分量决定左右：正=左侧，负=右侧
        let cross = tx * dy - ty * dx;
        let l = best_d2.sqrt() * cross.signum();
        (s, l)
    }

    /// Frenet → Cartesian: (s, l) → (x, y)
    pub fn to_cartesian(&self, s_query: f64, l: f64) -> (f64, f64) {
        // 找 s_query 落在的区间，插值出参考点与切向
        let mut i = 0;
        while i + 1 < self.s.len() && self.s[i + 1] < s_query {
            i += 1;
        }
        let (tx, ty) = self.tangent_at(i);
        // 法向量 = 切向逆时针转 90°：(-ty, tx)
        let (nx, ny) = (-ty, tx);
        let (bx, by) = self.pts[i]; // 简化：不在区间内再插值
        (bx + l * nx, by + l * ny)
    }
}

fn main() {
    // 一条向右弯的参考线
    let refline = ReferenceLine::new(vec![
        (0.0, 0.0), (10.0, 0.0), (20.0, 2.0), (30.0, 6.0), (40.0, 12.0),
    ]);
    // 车在世界坐标 (20, 4)，转到 Frenet
    let (s, l) = refline.to_frenet(20.0, 4.0);
    println!("Cartesian (20, 4) -> Frenet (s={:.2}, l={:.2})", s, l);
    // 再转回去验证闭环
    let (x, y) = refline.to_cartesian(s, l);
    println!("Frenet (s={:.2}, l={:.2}) -> Cartesian ({:.2}, {:.2})", s, l, x, y);
}
```

`to_frenet` 告诉你"沿路开了 s 米、偏离中心 l 米"，`to_cartesian` 把规划在 Frenet 里算好的轨迹转回世界坐标给控制。**一个完整的 Frenet 规划流水线就是**：障碍物和自车转进 Frenet → 纵向（s-t）和横向（l-s 或 l-t）分别规划（用五次/四次多项式或 QP）→ 合成轨迹 → 转回 Cartesian。

> **真实项目里**：上面的转换为了教学做了简化（最近点用暴力搜、法向用差分）。生产代码要处理一堆恶心的边角：参考线要先**平滑**（原始地图中心线有噪声，直接求切向会抖）、最近点要在**线段上投影**而非只找采样点、弯道曲率大时 Frenet 会**畸变**（曲率半径内侧坐标会挤压甚至自交）。Apollo 的 `ReferenceLine` 和 `FrenetFrame` 类为此写了上千行。但**核心思想就是你上面看到的这几十行**——别被工程复杂度吓到，思想是简单的。

## 工程现实：实时性、可行性、怎么调

算法讲完，说点让你在项目里不挨骂的实话。

**实时性是铁律。** 规划一般跑 **10~20 Hz**，意味着每一轮必须在 **50~100 ms** 内出解，且要留裕量给通信和控制。这直接淘汰了很多"理论最优但慢"的方法。所以你会看到大量工程妥协：A\* 用加权版换速度、优化限制迭代次数、候选轨迹数量卡上限、参考线只取前方一段。**"次优但按时"永远优于"最优但超时"**——超时的规划器等于没有规划器，车会拿着上一帧的旧轨迹裸奔。

**可行性 = 动力学约束。** 规划出的轨迹车必须真能开出来。三条硬线：曲率不能超过最小转弯半径的倒数（方向盘打不了那么急）、横向加速度不能超过轮胎抓地极限（`a_lat = v²·κ`，速度快时曲率就得小，否则打滑）、加速度和 jerk 要在舒适/物理范围内。一条几何上很美但要求 `2g` 横向加速度的轨迹，控制根本跟不上，等于废纸。**规划必须自己吃掉动力学约束，不能甩锅给控制。**

**兜底与安全。** 和预测一样，规划也要有退路。主规划器无解或超时时，要有一个**保底轨迹**——通常是"沿当前车道舒适减速直至停车"的紧急轨迹。宁可停下，不可失控。

**在 CARLA / 仿真里怎么调（呼应第九部分）。** 规划最难的不是写出来，是**调参**——那一堆代价权重 `w_smooth / w_ref / w_safe` 之间的平衡，安全权重太大车龟缩不敢动，效率权重太大车贴着障碍飙。工程上的做法是在 **CARLA 等仿真**里搭一批**标准场景**（跟车、超车、行人横穿、路口汇入——正是 1.3 节那个行人场景的加强版），每改一次权重就跑全套场景回归，看有没有"修好一个场景弄坏三个"。人的直觉调不动这么多耦合参数，**场景化的自动回归**才是规划工程师真正的日常。CARLA 提供可复现的场景和真值，让你能量化"这次改动让急刹率降了多少、让通行效率变了多少"——没有仿真闭环，调规划就是盲人摸象。

> **中高级视角**：面试问"你怎么保证规划的轨迹安全"，初级答"我加了避障代价项"，高级答"我有三层：优化里的软约束（代价）尽量远离障碍、硬约束（碰撞检测）绝不采纳撞的轨迹、以及独立于主规划器的**安全监控层**在最后一道关卡校验并可触发保底减速"。安全从来不靠单点，靠纵深防御——这个思路贯穿整个自动驾驶。

## 小结

- 规划分**三层**：路由（全局图搜索）→ 行为决策（离散，下一章）→ 运动规划（几十毫秒出一条具体轨迹）。
- **路径**只有空间、**轨迹**多了时间；控制要的是轨迹。处理二者关系有"路径-速度分解"和"时空联合"两派。
- 四大方法家族：**图搜索**（A\*/Hybrid A\*，给了可运行 A\* 和 Hybrid A\* 骨架）、**采样**（RRT/RRT\*）、**曲线**（五次多项式，给了完整推导与可运行代码）、**优化**（把规划写成带约束的 QP，lattice / EM planner，给了极简 lattice）。
- **Frenet（SL）坐标系**是现代规划的地基：沿参考线把弯路"拉直"，纵横解耦，让五次多项式和 QP 都能优雅工作——给了 Cartesian↔Frenet 的可运行转换代码。
- 工程铁律：**实时性**（次优但按时 > 最优但超时）、**可行性**（自己吃掉动力学约束）、**兜底**（保底减速轨迹）、**场景化仿真回归**（在 CARLA 里调那堆代价权重）。

我们已经会算一条轨迹了，但一直回避了上游那个离散的、更像"人在思考"的问题：到底是**跟车还是超车、让行还是通过、变道还是等待**？这些不是解方程能解出来的，是**决策**。下一章我们讲有限状态机、行为树、博弈，并给一个红绿灯 + 行人场景的完整决策实现，把 1.3 节那个悬着的行人彻底了结。
