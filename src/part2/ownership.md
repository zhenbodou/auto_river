# 2.2 所有权、借用与生命周期

这是本书第一个真正的硬骨头，也是最关键的一章。所有权（ownership）是 Rust 的灵魂——正是它让 Rust 敢承诺"编译期消灭内存错误和数据竞争"。学会它，后面一切都顺；绕过它，你永远在和编译器打游击战。

我要坦白一件事：**几乎每个 Rust 学习者都会经历一段"和借用检查器搏斗"的痛苦期**。你会写出你觉得天经地义的代码，编译器却红着脸拒绝。这一章我不会假装这个过程不存在——恰恰相反，我会带你**故意写错、看编译器报什么、然后学会怎么改**。因为那些报错信息里藏着 Rust 想教给你的世界观。熬过这一章，借用检查器就从你的敌人变成你的朋友。

我们全程用一个智驾里再真实不过的东西做例子：**一帧点云（point cloud）和它衍生出的障碍物列表的内存管理**。这比"待办事项 App"贴近你的真实工作一万倍，而且点云够大——十几万个点、几 MB——大到让"要不要拷贝一份"成为一个**性能生死问题**，而不是学院派的细节。

## 先问一个 C++ 老问题：这块内存谁负责释放？

一帧激光雷达点云在内存里长这样：

```rust
#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

/// 一帧点云：十几万个点
pub struct PointCloud {
    pub timestamp: f64,
    pub points: Vec<Point>,
}
```

一帧 12 万个点，每个点 16 字节，光 `points` 就是约 2 MB。这块内存从传感器驱动产生，流经预处理、地面分割、聚类、送进感知网络……一路上被无数函数碰到。

在 C++ 里，一个折磨了工程师几十年的问题是：**这块内存到底谁负责 `delete`？** 释放早了 → 别人还在用它 → 悬垂指针（use-after-free）。释放晚了或忘了 → 内存泄漏。两个地方都释放 → double free 崩溃。多线程同时读写 → 数据竞争。这些 bug 往往不可复现，在车上偶发，是安全攸关系统的噩梦（第 1.4 节说过）。

Rust 的答案简单到近乎粗暴：**每块内存，任意时刻，有且仅有一个所有者（owner）。所有者离开作用域，内存自动释放。** 没有 GC，没有手动 free，编译器在编译期就把这套规则钉死。

## 所有权三条铁律

记住这三条，你就掌握了 Rust 的一半：

1. **每个值有唯一的所有者。**
2. **同一时刻只能有一个所有者**（所有权可以"移动"给别人，但不能同时属于两个人）。
3. **所有者离开作用域时，值被 drop（析构 + 释放内存）。**

看第三条的实际效果：

```rust,ignore
fn process_frame() {
    let cloud = PointCloud {
        timestamp: 0.0,
        points: vec![Point { x: 0.0, y: 0.0, z: 0.0, intensity: 0.0 }; 120_000],
    };
    // ... 用 cloud 干活 ...
} // <- cloud 在这里离开作用域，那 2 MB 内存自动释放，无需你操心
```

函数结束，`cloud` 的作用域到头，编译器**自动**在这里插入释放代码。你不写 free，也不可能忘写。**这就是 RAII（资源获取即初始化）在 Rust 里的默认形态，而且是强制的。**

## 移动（move）：所有权的转让

现在关键点来了。看这段代码，它会让每个从 C++/Python 来的人愣一下：

```rust,ignore
fn main() {
    let cloud_a = make_cloud();       // cloud_a 拥有这帧点云
    let cloud_b = cloud_a;            // 所有权“移动”给 cloud_b
    println!("{}", cloud_a.timestamp); // ❌ 编译错误！
}
```

编译器报错：

```text
error[E0382]: borrow of moved value: `cloud_a`
 --> src/main.rs:4:20
  |
2 |     let cloud_a = make_cloud();
  |         ------- move occurs because `cloud_a` has type `PointCloud`,
  |                 which does not implement the `Copy` trait
3 |     let cloud_b = cloud_a;
  |                   ------- value moved here
4 |     println!("{}", cloud_a.timestamp);
  |                    ^^^^^^^ value borrowed here after move
```

**发生了什么？** `let cloud_b = cloud_a;` 这行**不是拷贝**，而是把所有权从 `cloud_a` **移动**到 `cloud_b`。移动之后，`cloud_a` 就"作废"了——你不能再用它。为什么这么设计？因为如果允许 `cloud_a` 和 `cloud_b` 同时有效，它们就都指向同一块 2 MB 内存，函数结束时两个都想释放它 → double free。Rust 从根上禁止了这种局面。

**这在性能上是巨大的好处**：移动只是把栈上那个"指针 + 长度 + 容量"的胖指针（对 `Vec` 而言 24 字节）搬过去，**那 2 MB 堆内存原地不动，一个字节都没拷**。C++ 的 `std::move` 想做同样的事，但它是"约定"，用错了（移动后又用原对象）编译器不拦你；Rust 的移动是编译器强制的，用错直接不给编译。

要真的复制那 2 MB，你得显式 `.clone()`：

```rust,ignore
let cloud_b = cloud_a.clone(); // 真的拷贝了 2 MB，cloud_a 依然有效
```

**看到这个 `.clone()` 就要警觉**：它在复制大内存。这是本章后面"Copy vs Clone 的性能"一节的核心。

### 函数调用也会移动

同样的规则适用于把值传进函数：

```rust,ignore
fn consume(cloud: PointCloud) {
    // cloud 被移动进来，函数结束时它在这里被 drop
}

fn main() {
    let cloud = make_cloud();
    consume(cloud);          // 所有权移动进 consume
    println!("{:?}", cloud); // ❌ 错误：cloud 已经被移走了
}
```

如果每次调函数都得交出所有权、用完还想用就得 `.clone()`，那 Rust 就没法用了——你会 clone 到天荒地老。所以 Rust 有了**借用**。

## 借用（borrow）：用一下，但不夺走所有权

**借用**就是"我借你的东西看一眼/改一下，用完还你，所有权始终是你的"。语法是 `&`：

```rust,ignore
/// 只读借用：数一帧点云里有多少个高点（可能是障碍物顶部）
fn count_high_points(cloud: &PointCloud) -> usize {
    cloud.points.iter().filter(|p| p.z > 1.0).count()
}

fn main() {
    let cloud = make_cloud();
    let n = count_high_points(&cloud); // 借用给函数，没有移动
    println!("高点数量: {}", n);
    println!("这帧时间戳: {}", cloud.timestamp); // ✅ cloud 还是我的，随便用
}
```

`&cloud` 创建一个**不可变引用（immutable reference）**，也叫**共享借用（shared borrow）**。函数拿它读数据，读完引用消失，`cloud` 毫发无损。**没有拷贝，没有转移所有权**——这才是处理大点云的正确姿势：几乎所有"只读一下"的函数都应该收 `&PointCloud` 而不是 `PointCloud`。

要修改，用**可变借用（mutable reference）** `&mut`：

```rust,ignore
/// 就地把点云平移（比如做坐标系补偿），不产生新的点云
fn translate(cloud: &mut PointCloud, dx: f32, dy: f32, dz: f32) {
    for p in cloud.points.iter_mut() {
        p.x += dx;
        p.y += dy;
        p.z += dz;
    }
}

fn main() {
    let mut cloud = make_cloud(); // 注意 mut
    translate(&mut cloud, 0.0, 0.0, -1.7); // 把点云降到地面坐标系
    // cloud 被就地修改，没有产生第二份 2 MB
}
```

## 借用检查器的核心规则（数据竞争死于此处）

现在是全章最重要的一条规则，请刻进脑子：

> **在任意时刻，对同一个数据，你要么可以有任意多个不可变借用（`&T`），要么只能有恰好一个可变借用（`&mut T`）。二者不可兼得。**

换句话说：**读可以共享，写必须独占。** 这条规则叫"共享 XOR 可变"（shared xor mutable）。

为什么这条规则能**从根上消灭数据竞争**？想想数据竞争的定义：两个线程同时访问同一内存，至少一个在写，且没有同步。Rust 说：你想写（`&mut`），就必须独占，别人连读都不行——那自然没有"同时访问、一个在写"的局面。**数据竞争在编译期就被这条规则拒绝了，而且不只是多线程，单线程的别名 bug（aliasing bug）也一并消灭。** 第 1.4 节那个 C++ vector 扩容导致悬垂引用的例子，根源正是"有人持有引用时，又有人改动了容器"——在 Rust 里这就是"共享借用还活着时试图可变借用"，编译器直接拒绝。

## 和编译器搏斗（一）：迭代时修改

来个真实会踩的坑。你想遍历障碍物列表，遇到置信度太低的就删掉：

```rust,ignore
fn filter_low_confidence(obstacles: &mut Vec<Obstacle>) {
    for (i, obs) in obstacles.iter().enumerate() { // 不可变借用 obstacles
        if obs.confidence < 0.5 {
            obstacles.remove(i);                    // ❌ 想可变借用 obstacles
        }
    }
}
```

编译器报错：

```text
error[E0502]: cannot borrow `*obstacles` as mutable because it is also
              borrowed as immutable
  |
2 |     for (i, obs) in obstacles.iter().enumerate() {
  |                     ---------------------------
  |                     |
  |                     immutable borrow occurs here
  |                     immutable borrow later used here
3 |         if obs.confidence < 0.5 {
4 |             obstacles.remove(i);
  |             ^^^^^^^^^^^^^^^^^^^ mutable borrow occurs here
```

`iter()` 借用了 `obstacles`（不可变），循环还没结束这个借用就活着；而 `remove` 需要可变借用。"共享 XOR 可变"被违反。**这不是 Rust 找茬——这个模式在 C++ 里就是经典 bug**：`remove` 可能导致 vector 内存重排，正在遍历的迭代器瞬间失效（iterator invalidation），后续访问就是未定义行为。Rust 只是在编译期把这个古老陷阱挡下了。

**怎么改？** 地道的 Rust 用 `retain`——一行搞定，还更快：

```rust,ignore
fn filter_low_confidence(obstacles: &mut Vec<Obstacle>) {
    obstacles.retain(|obs| obs.confidence >= 0.5);
}
```

`retain` 保留满足条件的元素、原地删掉其余，内部用一次遍历完成，没有反复的元素搬移。**编译器逼你写出了更好的代码。** 这是你会反复体会到的：借用检查器拒绝的往往真的是坏代码，改对之后常常更优雅、更快。

## 和编译器搏斗（二）：悬垂引用

你想写个函数，从点云里找到最近的点，返回它的引用：

```rust,ignore
fn nearest_point(cloud: PointCloud) -> &Point { // ❌
    cloud.points.iter()
        .min_by(|a, b| dist(a).partial_cmp(&dist(b)).unwrap())
        .unwrap()
}
```

编译器报错（简化）：

```text
error[E0106]: missing lifetime specifier
  |
1 | fn nearest_point(cloud: PointCloud) -> &Point {
  |                                        ^ expected named lifetime parameter
```

更深的问题是：`cloud` 是**按值传入**的，函数结束时它被 drop，那 2 MB 内存释放了。你却想返回一个指向它内部某个点的引用——这个引用会指向已释放的内存，**经典悬垂引用**。C++ 里这段能编过，运行时随机翻车。Rust 直接不让你编。

改法一：既然要返回内部引用，就得**借用**输入而不是拿走它：

```rust,ignore
fn nearest_point(cloud: &PointCloud) -> &Point {
    cloud.points.iter()
        .min_by(|a, b| dist(a).partial_cmp(&dist(b)).unwrap())
        .unwrap()
}
```

现在能编过了。这里编译器悄悄帮你做了件事——它推断出"返回的 `&Point` 活得和输入的 `&PointCloud` 一样久"。这个"活多久"，就是**生命周期（lifetime）**。

## 生命周期标注：给引用贴上"能活多久"的标签

生命周期是 Rust 里最吓人的语法，但概念其实朴素：**它不改变任何东西的实际存活时间，只是让你把"这个引用不能比它指向的数据活得更久"这件事写出来给编译器核对。**

大多数时候编译器能自己推断（这叫生命周期省略，lifetime elision），你根本不用写。只有当它推不出来、有歧义时，才要你手动标注。经典场景：函数返回的引用，可能来自两个输入之一，编译器不知道该绑给谁。

```rust,ignore
/// 从两帧点云里返回点数更多的那一帧的引用
fn denser<'a>(a: &'a PointCloud, b: &'a PointCloud) -> &'a PointCloud {
    if a.points.len() >= b.points.len() { a } else { b }
}
```

`'a`（读作 "tick a"）是一个**生命周期参数**。这个签名在说："输入 `a`、`b` 和返回值共享同一个生命周期 `'a`；返回的引用最多活到 `a` 和 `b` 里先失效的那一个为止。" 这样调用方就知道：只要它还想用返回值，就得保证 `a` 和 `b` 两帧点云都还活着。编译器用这个契约在调用点做检查，杜绝悬垂。

看它怎么拦住错误：

```rust,ignore
fn main() {
    let frame1 = make_cloud();
    let result;
    {
        let frame2 = make_cloud();
        result = denser(&frame1, &frame2); // result 借了 frame2
    } // ❌ frame2 在这里被 drop
    println!("{}", result.timestamp);      // 但 result 想活到这里
}
```

编译器会说 `frame2` 活得不够久（"borrowed value does not live long enough"）。**没有生命周期标注，编译器就没法证明 `result` 不会悬垂；有了它，编译器把这个偶发的、致命的 bug 变成一个明确的编译错误。** 这就是那句老话的含义：*Rust 把运行时的偶发崩溃，变成编译时的确定报错。*

> **中高级视角**：生命周期标注不产生任何运行时代码——它是纯粹的编译期契约，零成本。很多人怕它，其实 90% 的日常代码靠省略规则根本不用写生命周期。你真正需要手写它的地方，通常是在设计**持有引用的结构体**时。比如一个"点云的只读视图"：
>
> ```rust,ignore
> /// 一段点云的切片视图，不拥有数据，只借用
> struct CloudView<'a> {
>     timestamp: f64,
>     points: &'a [Point], // 借用别人的点，自己不持有那 2 MB
> }
> ```
>
> `CloudView<'a>` 在说"我这个结构体不能比我借的那段 `points` 活得更久"。这种"零拷贝视图"在高性能感知里非常常见——你想把一大帧点云按区域切成好几块分给不同线程处理，又不想复制数据，`&[Point]` 切片视图正是答案。

## Copy vs Clone：大点云不能随便 clone

现在回到那个贯穿全章的性能主题。你可能已经注意到：`Point` 派生了 `Copy`，而 `PointCloud` 只派生了 `Clone`。这个区别是有意的，背后是一条重要的性能纪律。

**`Copy`**：一个类型如果是 `Copy` 的，赋值和传参时会**按位复制**，而且原值**依然有效**（不发生移动）。只有"小、且完全在栈上、没有堆资源"的类型才配当 `Copy`——比如 `i32`、`f64`、`bool`，以及我们的 `Point`（就 16 字节，纯栈数据）。

```rust,ignore
let p1 = Point { x: 1.0, y: 2.0, z: 3.0, intensity: 0.5 };
let p2 = p1;           // Copy：拷贝 16 字节
println!("{}", p1.x);  // ✅ p1 依然有效，因为它是 Copy
```

**`Clone`**：显式的、可能昂贵的复制。任何类型都能实现 `Clone`，包括那些持有堆内存的（`Vec`、`String`、`PointCloud`）。它**不会**在赋值时自动发生——你必须亲手写 `.clone()`。这个"必须显式"的设计是刻意的：**Rust 想让每一次昂贵的深拷贝都在代码里显眼地暴露出来**，而不是像 C++ 拷贝构造函数那样悄无声息地发生。

对比 C++：一个 `std::vector<Point> a = b;` 就默默拷了整个 vector，你一眼看代码根本发现不了这里有 2 MB 的拷贝。Rust 里同样的操作要么是移动（不拷），要么你得写出 `b.clone()`（拷，但你看得见）。**代码审查时，`.clone()` 就是性能审查的抓手。**

看一个真实的性能陷阱：

```rust,ignore
// ❌ 反模式：每一帧都克隆整份点云，2 MB × 帧率 的无谓拷贝
fn process_bad(cloud: PointCloud) -> Vec<Obstacle> {
    let filtered = cloud.clone();       // 白拷 2 MB！
    let ground_removed = filtered.clone(); // 又白拷 2 MB！
    detect(&ground_removed)
}

// ✅ 正确：全程借用，零拷贝
fn process_good(cloud: &PointCloud) -> Vec<Obstacle> {
    detect(cloud) // 需要修改时才 &mut，需要新数据时才构造新的，绝不无脑 clone
}
```

在 10 Hz 的激光雷达上，`process_bad` 每秒白白拷贝 40 MB，吃掉内存带宽、增加延迟抖动。在讲究实时性和资源受限的车载环境里，这种"随手 clone"是初级工程师最常见、也最容易被 code review 揪出的性能问题。

> **面试题**：什么样的类型应该派生 `Copy`？为什么 `Vec<T>` 不能是 `Copy`？
> **答**：只有语义上"廉价按位复制、且没有需要特殊清理的资源"的小类型才该是 `Copy`，比如坐标、标志位。`Vec<T>` 持有堆内存，若它是 `Copy`，`let b = a;` 就会按位复制那个胖指针，让 `a` 和 `b` 指向**同一块**堆内存，两者析构时 double free——所以 Rust 从类型层面禁止 `Vec` 成为 `Copy`。这也正是"移动"存在的理由。

> **真实项目里**：处理点云的性能纪律可以总结成几条：① 默认传 `&PointCloud`；② 要改就传 `&mut PointCloud` 就地改；③ 真需要并行处理不同区域，用 `&[Point]` 切片视图分发，而不是切成多个 `Vec` 拷贝；④ 只有当你确实要产出一份"新的、独立的、要跨越原数据生命周期存活"的数据时，才 clone。你手边的 `inf/` 推理 SDK 把 Python 流水线移植成 Rust，能拿到性能收益，很大一部分正来自这种"数据只在管线里流动、靠移动和借用而非拷贝传递"的所有权设计——Python 里那些隐式的对象引用和拷贝，在 Rust 里变成了显式、可控、零拷贝的数据流。

## 一个综合例子：一帧点云的完整流转

把这一章的东西串起来，看一帧点云从进来到产出障碍物的所有权流转：

```rust,ignore
fn perception_pipeline(mut cloud: PointCloud) -> Vec<Obstacle> {
    // 1. cloud 的所有权移动进来。我们要就地改它，所以用 mut。

    // 2. 去地面：就地修改，可变借用，零拷贝
    remove_ground(&mut cloud);

    // 3. 统计信息：只读，不可变借用
    let n = count_high_points(&cloud);
    log::debug!("去地面后剩 {} 个高点", n);

    // 4. 聚类 + 检测：只读借用点云，产出全新的障碍物列表
    let obstacles = detect(&cloud);

    // 5. 函数结束，cloud 离开作用域，那 2 MB 在此自动释放
    obstacles
    // 障碍物列表的所有权移动给调用方
}
```

数一数：整个流水线里，那 2 MB 点云**只存在一份**，从头到尾没有被拷贝过。它被就地修改、被只读借用、最后自动释放。障碍物列表是新产出的，所有权干净地交还给调用方。**没有 GC，没有手动 free，没有一次多余的深拷贝，也不可能有悬垂或竞争**——这就是所有权系统给你的东西。

## 小结

- 所有权三铁律：**唯一所有者、所有权可移动不可共享、所有者离开作用域自动释放**。这是"没有 GC 也没有手动 free"的根基。
- **移动（move）** 只搬栈上的胖指针、不碰堆数据，是零成本的；**克隆（clone）** 才真拷贝堆内存，且必须显式写出来——这让每次昂贵拷贝在代码里都显眼可查。
- **借用**让你"用一下不夺走"：`&T` 共享只读、`&mut T` 独占可写。核心规则"**共享 XOR 可变**"从根上消灭数据竞争和迭代器失效这类别名 bug。
- **生命周期** 是零成本的编译期契约，把"引用不能比数据活得久"写给编译器核对，把运行时的偶发悬垂变成编译时的确定报错；日常大多靠省略规则不用手写，设计"持有引用的结构体/视图"时才需要。
- **性能纪律**：大点云默认传 `&`、要改传 `&mut`、并行用切片视图，**绝不无脑 clone**——`.clone()` 是 code review 里的性能抓手。
- 借用检查器拒绝的往往真是坏代码，改对之后通常更优雅更快。和它搏斗是暂时的，之后它是你的朋友。

下一章我们上 Rust 的类型系统——用 `enum` 优雅地建模传感器消息和障碍物类别，用 `trait` 定义"传感器""检测器"这样的抽象接口，并搞清一个中高级必考点：**动态分发（dyn）和泛型该怎么选**。
