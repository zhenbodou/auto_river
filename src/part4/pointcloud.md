# 4.4 点云处理与激光雷达感知

上一章我们在图像的 2D 世界里跑完了检测。现在切换到激光雷达（LiDAR）的 3D 世界。相机给的是稠密、有颜色但没深度的像素；激光雷达给的是稀疏、无颜色但**每个点都带精确 3D 坐标**的点云（point cloud）。1.3 节说过，一圈激光雷达点云约 12 万个点，每个点 `(x, y, z, intensity)`。这一章我们就来处理这十几万个点：怎么高效遍历、怎么下采样、怎么把地面剥掉、怎么把剩下的点聚成一个个障碍物，最后简单说说 3D 检测网络和 BEV。

这是重点章，也是**算法密度最高**的一章。点云处理是很多传统几何算法的主场，即便深度学习盛行的今天，RANSAC 地面分割、欧式聚类这些经典算法仍然活跃在量产系统里——因为它们**不需要训练、可解释、兜得住长尾**。你手写一遍，对激光雷达感知的理解会上一个台阶。

## 点云的数据结构与遍历性能

一个点最朴素的表示：

```rust
#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32, // 反射强度，0~1 或 0~255，材质相关
}

pub type PointCloud = Vec<Point>;
```

看起来平平无奇，但这里有个**决定性能的关键抉择**：用 **AoS（Array of Structs，结构体数组）** 还是 **SoA（Struct of Arrays，数组的结构体）**？

- **AoS**：`Vec<Point>`，就是上面这样。每个点的 xyz+intensity 挨在一起。**遍历单个点很方便**，缓存友好（处理一个点时它的四个字段都在一条缓存线上）。
- **SoA**：`struct Cloud { xs: Vec<f32>, ys: Vec<f32>, zs: Vec<f32>, is: Vec<f32> }`。所有点的 x 挨在一起。**批量只算某一维时极快**（比如"只算所有点的平均 z"能连续扫一整个 `Vec<f32>`，SIMD 友好）。

```rust
/// SoA 布局：某些批量运算（尤其能 SIMD 的）会明显更快
pub struct CloudSoA {
    pub xs: Vec<f32>,
    pub ys: Vec<f32>,
    pub zs: Vec<f32>,
    pub is: Vec<f32>,
}
```

> **中高级视角**：AoS vs SoA 是点云工程的第一个性能分水岭。十几万点、每帧 10 Hz，任何 `O(n)` 的操作都要认真对待内存布局。经验法则：**逻辑上"一个点一个点处理"用 AoS，数值上"整列一起算"用 SoA**。PCL（C++ 点云库）内部大量用 SoA + SIMD。本章为了代码清晰用 AoS，但你要知道生产里这块会为了性能变形。另一个大坑是：**永远不要在热循环里 clone 整个点云**——十几万个点的拷贝，每帧来一次就是几毫秒没了。用引用、切片、原地修改。

遍历时优先用迭代器，既地道又常常让编译器帮你优化：

```rust,ignore
/// 算点云的轴对齐包围盒（AABB），一次遍历。
fn bounding_box(cloud: &[Point]) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for p in cloud {
        min[0] = min[0].min(p.x); max[0] = max[0].max(p.x);
        min[1] = min[1].min(p.y); max[1] = max[1].max(p.y);
        min[2] = min[2].min(p.z); max[2] = max[2].max(p.z);
    }
    (min, max)
}
```

## 体素栅格下采样（voxel grid）

十几万个点，很多是冗余的——近处地面密密麻麻挤着几千个点，其实用不着这么多。**体素栅格下采样（voxel grid downsampling）**把 3D 空间切成一个个小立方体（**体素 voxel**，比如边长 0.2 米），**每个体素里的所有点用它们的质心（重心）代替一个点**。这样点数能砍到十分之一甚至更少，后续算法快得多，且几乎不损失结构信息。

**算法**：

1. 选体素边长 `leaf_size`。
2. 对每个点，算它落在哪个体素：`voxel = (floor(x/leaf), floor(y/leaf), floor(z/leaf))`，这个三元组当 key。
3. 用哈希表把点按体素分组，累加每个体素里点的坐标和计数。
4. 每个体素输出一个点：质心 = 坐标和 / 计数。

Rust 实现：

```rust,ignore
use std::collections::HashMap;

/// 体素栅格下采样。leaf_size 是体素边长（米）。
pub fn voxel_downsample(cloud: &[Point], leaf_size: f32) -> Vec<Point> {
    assert!(leaf_size > 0.0);
    let inv = 1.0 / leaf_size;

    // key: 体素整数坐标；value: (x和, y和, z和, i和, 计数)
    let mut voxels: HashMap<(i64, i64, i64), (f64, f64, f64, f64, u32)> = HashMap::new();

    for p in cloud {
        let key = (
            (p.x * inv).floor() as i64,
            (p.y * inv).floor() as i64,
            (p.z * inv).floor() as i64,
        );
        let e = voxels.entry(key).or_insert((0.0, 0.0, 0.0, 0.0, 0));
        e.0 += p.x as f64;
        e.1 += p.y as f64;
        e.2 += p.z as f64;
        e.3 += p.intensity as f64;
        e.4 += 1;
    }

    voxels
        .into_values()
        .map(|(sx, sy, sz, si, n)| {
            let n = n as f64;
            Point {
                x: (sx / n) as f32,
                y: (sy / n) as f32,
                z: (sz / n) as f32,
                intensity: (si / n) as f32,
            }
        })
        .collect()
}
```

工程细节：

- **`leaf_size` 是精度/速度的旋钮**。太大（比如 0.5 米）会把小物体（远处行人）抹平甚至整个吞掉——下游就漏检了。太小则降采样效果不明显。城市场景常用 0.1~0.2 米。
- **可以用体素中心代替质心**（`key * leaf + leaf/2`），更快（不用累加坐标），但点会有栅格化的"网格感"。质心更平滑、更贴合真实表面，多数场景更好。
- **哈希是瓶颈**：十几万次哈希插入。生产里会换更快的哈希（`ahash`/`fxhash`）或直接把体素坐标编码成一个整数 key。别用默认的 `SipHash`（抗碰撞但慢）。

## 地面分割：RANSAC 平面拟合

点云里绝大部分点是**地面**。地面点对"找障碍物"毫无用处，反而碍事——聚类时地面会把所有物体连成一坨。所以经典流水线的第一步几乎都是**地面分割（ground segmentation）**：把地面点剥离出去，只留下"地面之上"的点做后续聚类。

最经典的方法是 **RANSAC（Random Sample Consensus，随机采样一致性）拟合平面**。核心思想极其优雅：**地面近似是一个平面，我随机猜一个平面，看有多少点贴着它；猜很多次，贴合点最多的那个平面就是地面。**

### 数学：平面与点到平面距离

一个平面用方程表示：

$$
ax + by + cz + d = 0
$$

其中 `(a, b, c)` 是平面的**法向量（normal）**（垂直于平面的方向），通常归一化到 `a²+b²+c²=1`。对地面，法向量应该接近竖直方向 `(0,0,1)`。

任意一点 `(x0, y0, z0)` 到这个平面的**垂直距离**是：

$$
\text{dist} = \frac{|a x_0 + b y_0 + c z_0 + d|}{\sqrt{a^2+b^2+c^2}}
$$

若法向量已归一化，分母是 1，距离就是分子的绝对值。这个距离就是判断"点贴不贴平面"的尺子。

### 怎么从 3 个点定一个平面

RANSAC 每次随机取 3 个点（3 点定一平面）。给定三点 `P1, P2, P3`，法向量是两条边向量的**叉积（cross product）**：

$$
\vec{n} = (P_2 - P_1) \times (P_3 - P_1)
$$

叉积得到的向量同时垂直于两条边，也就垂直于平面。归一化后就是 `(a,b,c)`，再用 `d = -(a x_1 + b y_1 + c z_1)` 定出 `d`。

### RANSAC 算法

1. 迭代 K 次：
   - 随机取 3 个点，算出候选平面 `(a,b,c,d)`。
   - 数有多少点到该平面距离 < 阈值 `dist_thresh`（这些叫**内点 inliers**）。
   - 记住内点最多的那个平面。
2. 用内点最多的平面，把所有内点判为地面，其余为非地面。

**为什么迭代 K 次就够？** 概率论。设内点比例是 `w`，随机取 3 个点全是内点的概率是 `w³`。取 K 次至少有一次全命中的概率是 `1 - (1-w³)^K`。地面点通常占一大半（`w≈0.6`），K 取 50~100 就有极高概率抽中一次"干净"的地面三点样本。这就是 RANSAC 便宜又鲁棒的原因——它对离群点（outliers，比如障碍物点）天然免疫。

Rust 实现：

```rust,ignore
use rand::seq::SliceRandom;

#[derive(Debug, Clone, Copy)]
pub struct Plane {
    pub a: f32,
    pub b: f32,
    pub c: f32,
    pub d: f32,
}

impl Plane {
    /// 点到平面的距离（假设法向量已归一化）。
    fn dist(&self, p: &Point) -> f32 {
        (self.a * p.x + self.b * p.y + self.c * p.z + self.d).abs()
    }
}

/// 由三点确定一个平面（叉积求法向量）。三点共线则返回 None。
fn plane_from_3(p1: &Point, p2: &Point, p3: &Point) -> Option<Plane> {
    let v1 = [p2.x - p1.x, p2.y - p1.y, p2.z - p1.z];
    let v2 = [p3.x - p1.x, p3.y - p1.y, p3.z - p1.z];
    // 叉积 v1 × v2
    let n = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ];
    let norm = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if norm < 1e-6 {
        return None; // 三点共线，构不成平面
    }
    let (a, b, c) = (n[0] / norm, n[1] / norm, n[2] / norm);
    let d = -(a * p1.x + b * p1.y + c * p1.z);
    Some(Plane { a, b, c, d })
}

/// RANSAC 地面分割。返回 (地面点索引集合是内点掩码, 拟合出的平面)。
pub fn ransac_ground(
    cloud: &[Point],
    iters: usize,
    dist_thresh: f32,
) -> (Vec<bool>, Option<Plane>) {
    let mut rng = rand::thread_rng();
    let idxs: Vec<usize> = (0..cloud.len()).collect();
    let mut best_inliers = 0usize;
    let mut best_plane: Option<Plane> = None;

    for _ in 0..iters {
        // 随机取 3 个不同的点
        let sample: Vec<usize> = idxs.choose_multiple(&mut rng, 3).copied().collect();
        let plane = match plane_from_3(&cloud[sample[0]], &cloud[sample[1]], &cloud[sample[2]]) {
            Some(p) => p,
            None => continue,
        };
        // 可选：只接受近似水平的平面（法向量接近竖直），排除墙面等竖直平面
        if plane.c.abs() < 0.85 {
            continue;
        }
        // 数内点
        let count = cloud.iter().filter(|p| plane.dist(p) < dist_thresh).count();
        if count > best_inliers {
            best_inliers = count;
            best_plane = Some(plane);
        }
    }

    // 用最优平面标记地面点
    let mask = match &best_plane {
        Some(pl) => cloud.iter().map(|p| pl.dist(p) < dist_thresh).collect(),
        None => vec![false; cloud.len()],
    };
    (mask, best_plane)
}
```

工程现实：

- **`plane.c.abs() < 0.85` 那行很重要**：它拒绝掉墙、大卡车侧面这类竖直大平面被误当成"地面"。地面的法向量该接近 `(0,0,±1)`，即 `|c|≈1`。少了这道约束，RANSAC 可能拟合出一面墙。
- **`dist_thresh` 典型 0.1~0.3 米**：太小地面剥不干净（残留地面点会干扰聚类），太大会把贴地的矮障碍物（路沿、减速带）当地面吃掉。
- **单平面假设会失效**：上下坡、起伏路面不是一个平面。生产里常把点云按距离切成多个扇区/环，**分段拟合多个局部平面**，或用更复杂的方法（如基于射线的、或专门的地面网络）。RANSAC 单平面是够用且好用的基线。
- **实时性**：`iters=100`、每次 O(n) 数内点，是 `100n`。十几万点跑得动，但如果先 voxel 下采样再 RANSAC，会快很多且精度几乎不掉——**这就是为什么下采样几乎总在地面分割之前**。

> **面试题**：RANSAC 为什么对离群点鲁棒，而最小二乘拟合不行？
> **答**：最小二乘让"所有点的误差平方和最小"，一个远处的离群点（比如一根伸到天上的电线杆点）会因为误差被平方而**极大地拉偏**整个拟合结果。RANSAC 不看所有点，它找"支持者最多"的模型——离群点天生进不了内点集，对结果零影响。代价是它是随机的、要迭代、不保证每次结果完全一致。鲁棒性 vs 确定性，这是经典权衡。

## 欧式聚类：把点聚成物体（kd-tree 加速）

剥掉地面后，剩下的点是一堆悬在空中的"障碍物点"。但点云不知道"哪些点属于同一辆车"。**欧式聚类（Euclidean clustering）**解决这个：**空间上挨得足够近（距离 < 阈值）的点，归为同一个物体。**

朴素想法是对每个点找它半径 `r` 内的所有邻居，做区域生长（region growing）。但"找半径内邻居"如果暴力两两比较是 O(n²)，十几万点直接爆炸。**必须用空间索引加速**——这就是 **kd-tree** 登场的地方。

### kd-tree 是什么

kd-tree（k-dimensional tree）是一种把 k 维空间递归二分的树。它让"找某点附近的邻居"从 O(n) 降到平均 O(log n)。对点云的半径搜索、最近邻搜索，它是标配。Rust 生态里有现成的：

```toml
# Cargo.toml
[dependencies]
kiddo = "4"      # 高性能 kd-tree，支持半径/最近邻查询
# 或 kdtree = "0.7"，API 更朴素
rand = "0.8"
```

`kiddo` 是目前 Rust 里性能很好的 kd-tree 实现，专为大规模最近邻/半径查询优化。下面用它做欧式聚类。

### 算法：基于 kd-tree 的区域生长

1. 把所有非地面点建进 kd-tree。
2. 维护一个"已访问"标记。对每个还没访问的点，开一个新簇（cluster），做广度优先扩张：
   - 把它入队、标记已访问。
   - 队列里取一个点，用 kd-tree 查它半径 `tol` 内的所有邻居，把没访问过的邻居加进当前簇、入队、标记。
   - 队列空了，一个簇就长完了。
3. 簇大小在 `[min_pts, max_pts]` 之间的才保留（太小是噪声，太大可能是没剥干净的地面或墙）。

```rust,ignore
use kiddo::{KdTree, SquaredEuclidean};
use std::collections::VecDeque;

/// 欧式聚类。tol 是邻域半径（米），返回若干簇，每个簇是点索引列表。
pub fn euclidean_cluster(
    cloud: &[Point],
    tol: f32,
    min_pts: usize,
    max_pts: usize,
) -> Vec<Vec<usize>> {
    // 1) 建 kd-tree。kiddo 用 [f32; 3] 作为坐标，usize 作为负载(点索引)
    let mut tree: KdTree<f32, 3> = KdTree::new();
    for (i, p) in cloud.iter().enumerate() {
        tree.add(&[p.x, p.y, p.z], i as u64);
    }

    let tol_sq = tol * tol; // kiddo 的 SquaredEuclidean 返回平方距离
    let mut visited = vec![false; cloud.len()];
    let mut clusters: Vec<Vec<usize>> = Vec::new();

    for seed in 0..cloud.len() {
        if visited[seed] {
            continue;
        }
        // 2) 从 seed 开始区域生长（BFS）
        let mut cluster = Vec::new();
        let mut queue = VecDeque::new();
        queue.push_back(seed);
        visited[seed] = true;

        while let Some(cur) = queue.pop_front() {
            cluster.push(cur);
            let p = &cloud[cur];
            // 半径搜索：返回 tol 内所有点
            let neighbors = tree.within::<SquaredEuclidean>(&[p.x, p.y, p.z], tol_sq);
            for nb in neighbors {
                let idx = nb.item as usize;
                if !visited[idx] {
                    visited[idx] = true;
                    queue.push_back(idx);
                }
            }
        }

        // 3) 按大小过滤
        if cluster.len() >= min_pts && cluster.len() <= max_pts {
            clusters.push(cluster);
        }
    }
    clusters
}
```

> 注：`kiddo` 具体 API（`within` 的签名、距离度量类型）随版本略有变化，以你锁定版本的文档为准；这里展示的是"建树 → 逐点半径查询 → 区域生长"的骨架逻辑，思路是通用的。

每个簇就是一个候选障碍物。算一下它的 AABB（用前面的 `bounding_box`），就得到了一个粗糙的 3D 框——位置、大小都有了：

```rust,ignore
/// 把一个簇变成一个粗糙的 3D 障碍物框
fn cluster_to_obstacle(cloud: &[Point], cluster: &[usize]) -> Obstacle {
    let pts: Vec<Point> = cluster.iter().map(|&i| cloud[i]).collect();
    let (min, max) = bounding_box(&pts);
    let center = [
        ((min[0] + max[0]) / 2.0) as f64,
        ((min[1] + max[1]) / 2.0) as f64,
        ((min[2] + max[2]) / 2.0) as f64,
    ];
    let size = [
        (max[0] - min[0]) as f64,
        (max[1] - min[1]) as f64,
        (max[2] - min[2]) as f64,
    ];
    Obstacle {
        id: 0,             // 待跟踪赋值（4.5）
        class: ObstacleClass::Unknown, // 几何聚类不知道类别！
        position: center,
        velocity: [0.0; 3], // 待跟踪估计
        size,
        heading: 0.0,
        confidence: 0.8,
    }
}
```

工程现实，句句血泪：

- **`class` 是 `Unknown`**：纯几何聚类**不知道这堆点是车是人还是垃圾桶**。要类别，要么靠点云分类网络，要么靠和相机检测融合（4.5）。这是几何方法的根本局限。
- **`tol` 的两难**：太小，一辆车会被切成好几块（欠聚类）；太大，两个挨近的行人会被粘成一坨（过聚类）。经典失败场景：**两个人并肩走**被聚成一个"大障碍物"，或者**一辆公交车**因为点稀被切成三段。
- **距离自适应**：激光雷达点云**近密远疏**。固定 `tol` 对远处物体（点稀）会欠聚类。成熟系统用**随距离增大的 `tol`**，或先做距离归一化。
- **建树成本**：每帧重建 kd-tree 也要时间。十几万点建树 + 查询，配合前面的 voxel 下采样（先把点砍到几万）才能实时。

> **中高级视角**：voxel 下采样 → RANSAC 地面分割 → 欧式聚类，这条"传统三件套"是激光雷达感知的**经典基线**，直到今天很多量产系统仍保留它，作为深度学习检测的**兜底和补充**。为什么？因为它**不需要训练、对没见过的物体（长尾）天然有效**——一个从卡车上掉下来的奇形怪状的沙发，检测网络没见过可能漏检，但它是"地面之上的一坨点"，聚类照样能把它框出来。深度学习管"认识常见物体"，几何聚类管"别撞上任何一坨东西"，两者互补。这就是 4.1 节说的 freespace/几何方法作为"长尾安全网"的价值。

## 3D 检测网络：PointPillars 与 CenterPoint

传统聚类给不了类别、朝向也粗糙。现代激光雷达感知的主力是**3D 检测网络**——直接从点云端到端输出带类别、朝向、尺寸的 3D 框。你要知道两个代表：

- **PointPillars**：把点云在俯视方向切成一根根竖直的"柱子（pillar）"，每根柱子里的点用小网络编码成一个特征向量，于是整个点云变成一张**伪图像**（BEV 特征图），然后就能套用成熟的 2D 卷积检测头。**优点是快**——把 3D 问题降维成 2D，实时性好，是很多量产系统的选择。
- **CenterPoint**：不预测框的角点，而是**预测物体中心点的热力图**（哪里是一个物体的中心），再从中心回归尺寸、朝向、速度。基于中心点的表示对朝向和尺度更友好，精度更高，是近年的强基线。

它们的共同点：**都要先把稀疏无序的点云，变成某种规整的表示（pillar/voxel 特征、BEV 图）**，才能上卷积。这引出下一个关键概念——BEV。

## 点云到 BEV：鸟瞰图

**BEV（Bird's Eye View，鸟瞰图）**是激光雷达乃至整个现代感知的核心表示。思路是：**从正上方俯视，把 3D 空间压成一张 2D 俯视栅格图**。每个栅格格子（比如 0.2m × 0.2m）里统计落进来的点的特征（最高点、点数、平均强度等）。

为什么 BEV 这么重要：

1. **尺度不变**：在 BEV 里，一辆车不管远近，占的格子数差不多（因为是物理尺寸），不像图像里近大远小。这对检测极友好。
2. **没有遮挡叠加**：俯视图里物体不会像相机图那样前后重叠。
3. **天然是规划要的坐标系**：规划就在这个俯视的自车/地图平面里干活，BEV 直接对齐。
4. **易于多传感器融合**：相机特征、激光特征都能投到同一张 BEV 上对齐——这是"BEV 融合"成为当前主流的原因，也是特斯拉"占用网络"、以及大量最新研究的落脚点。

一个最简单的"点云 → BEV 高度图"实现，帮你建立直觉：

```rust,ignore
/// 把点云投成一张 BEV 高度图。范围 [x_min,x_max]×[y_min,y_max]，分辨率 res 米/格。
/// 每格记录落入点的最大高度 z（简单起见）。返回 (H, W) 的二维数组。
pub fn cloud_to_bev(
    cloud: &[Point],
    x_range: (f32, f32),
    y_range: (f32, f32),
    res: f32,
) -> ndarray::Array2<f32> {
    use ndarray::Array2;
    let w = ((x_range.1 - x_range.0) / res).ceil() as usize;
    let h = ((y_range.1 - y_range.0) / res).ceil() as usize;
    let mut bev = Array2::<f32>::from_elem((h, w), f32::NEG_INFINITY);

    for p in cloud {
        if p.x < x_range.0 || p.x >= x_range.1 || p.y < y_range.0 || p.y >= y_range.1 {
            continue;
        }
        let col = ((p.x - x_range.0) / res) as usize;
        let row = ((p.y - y_range.0) / res) as usize;
        if p.z > bev[[row, col]] {
            bev[[row, col]] = p.z; // 取最高点作为该格高度
        }
    }
    // 把没有点的格子的 -inf 归零
    bev.mapv_inplace(|v| if v.is_finite() { v } else { 0.0 });
    bev
}
```

真实的 BEV 编码远比这复杂（多通道：高度分层、密度、强度；或用网络学的 pillar 特征），但内核就是这个"3D 点 → 俯视 2D 栅格"的投影。**理解了这张图，你就理解了现代感知的通用语言。** 从 4.2 的图像张量，到这里的 BEV 张量，再到 4.3 的推理引擎——它们最终都汇成"把某种规整张量喂给网络"。

## 一条完整的传统激光雷达感知流水线

把本章的算法串起来，就是一条能跑的激光雷达感知基线：

```rust,ignore
fn lidar_perception(raw: &[Point]) -> Vec<Obstacle> {
    // 1) 下采样：十几万点 → 几万点，后续都快
    let ds = voxel_downsample(raw, 0.15);

    // 2) 地面分割：剥掉地面点
    let (ground_mask, _plane) = ransac_ground(&ds, 100, 0.2);
    let non_ground: Vec<Point> = ds.iter().zip(&ground_mask)
        .filter(|(_, &is_ground)| !is_ground)
        .map(|(p, _)| *p)
        .collect();

    // 3) 聚类：非地面点聚成物体
    let clusters = euclidean_cluster(&non_ground, 0.5, 15, 20_000);

    // 4) 每个簇 → 一个粗糙 Obstacle（类别 Unknown，待融合/跟踪细化）
    clusters.iter()
        .map(|c| cluster_to_obstacle(&non_ground, c))
        .collect()
}
```

这条流水线产出的 `Obstacle` 有位置、大小，但**没有类别、没有 ID、没有速度**。类别靠和相机检测融合补，ID 和速度靠跟踪补——这正是下一章 4.5 要干的全部事情。至此，感知的"看到"部分（图像检测、点云聚类/检测）我们都覆盖了，接下来是把这些逐帧、多源的观测，缝合成一个稳定、带 ID、带速度、带不确定性的世界模型。

## 小结

- 点云是十几万个 `(x,y,z,intensity)` 点；**AoS/SoA 内存布局**和"绝不在热循环里 clone"是性能第一课。
- **体素下采样**用体素质心代替一堆点，把点数砍一个数量级，几乎总放在流水线最前面；`leaf_size` 是精度/速度旋钮。
- **RANSAC 地面分割**：随机猜平面、数内点、留最优；对离群点鲁棒（不像最小二乘会被拉偏）；用叉积从 3 点定平面、点到平面距离当尺子；加"法向量近竖直"约束防止拟合到墙。
- **欧式聚类**用 **kd-tree（kiddo）**把半径搜索降到 O(log n)，区域生长把近邻点聚成物体；`tol` 太小欠聚类、太大过聚类，要距离自适应。几何聚类**给不了类别**，但**对长尾物体天然有效**，是深度学习的兜底。
- **3D 检测网络**（PointPillars 快、CenterPoint 准）和 **BEV 鸟瞰图**是现代激光雷达感知的主力；BEV 尺度不变、无遮挡叠加、对齐规划坐标系、易做多传感器融合，是当前感知的通用语言。
- 传统三件套流水线产出的 `Obstacle` **缺类别、ID、速度**——下一章补齐。

下一章 4.5，我们把逐帧、多源的检测结果，用跟踪和融合缝成一个稳定的世界模型：匈牙利算法做数据关联、卡尔曼滤波估速度、前融合 vs 后融合、以及怎么把 1.3 节承诺的"带不确定性的感知输出"真正用协方差落地。
