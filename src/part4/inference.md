# 4.3 深度学习推理：onnxruntime / tract / candle

这是第四部分的重点章。上一章我们把一张图像加工成了 `[1,3,640,640]` 的归一化张量，现在要把它真正喂进一个神经网络，跑出检测结果。问题是：**模型是 Python 用 PyTorch 训的，我们要在 Rust 里部署**。这两件事怎么衔接？Rust 里用什么跑推理？跑出来一堆原始数字怎么变成 `Vec<Obstacle>`？

这一章回答这些。我们先讲清"训练在 Python、部署在 Rust"这套行业标准分工，再讲 ONNX 这个把两者黏起来的交换格式，然后横向对比 Rust 三大推理方案（`ort` / `tract` / `candle`），给一个用 `ort` 跑 ONNX 的完整可运行骨架，最后把后处理里绕不开的 **NMS 非极大值抑制**用 Rust 从头实现。

## 训练在 Python，部署在 Rust：为什么这么分工

先接受一个现实：**你几乎不会用 Rust 训练模型**，至少现在不会。原因很简单——训练生态在 Python。PyTorch、数据加载、可视化、预训练权重、论文代码，全在 Python。算法工程师在 Python 里迭代模型，一天试十个想法。让他们用 Rust 训练，等于让他们戴着镣铐跳舞。

但**部署（inference/推理）是另一回事**。上车运行的代码要：

- **实时**：每帧几毫秒的预算，Python 的 GIL 和解释器开销是负担。
- **可靠**：车上不能有内存泄漏、不能有"跑几小时就崩"的隐患——这正是 Rust 的主场。
- **易集成**：要嵌进 C++/Rust 的主系统里，和感知的其他模块、中间件无缝对接。
- **无 Python 运行时**：车载环境往往不想背一整个 Python 解释器和一堆 `pip` 依赖。

于是行业形成了清晰的分工：

```text
   ┌─────────────── Python 世界 ───────────────┐
   │  数据 → PyTorch 训练 → 得到 model.pth      │
   │                    │                       │
   │                    ▼  torch.onnx.export    │
   │              model.onnx  ← 交换格式          │
   └────────────────────┬──────────────────────┘
                        │  (一个 .onnx 文件，跨语言)
   ┌────────────────────▼──────────────────────┐
   │   Rust / C++ 世界（部署）                    │
   │   前处理 → 推理引擎加载 model.onnx → 后处理   │
   │   ort / tract / candle / TensorRT           │
   └───────────────────────────────────────────┘
```

**你的职责边界**：算法同事交给你一个 `.onnx` 文件和一份"前处理/后处理说明"（输入尺寸、mean/std、输出格式），你在 Rust 里把它高效、正确地跑起来，产出 `Vec<Obstacle>`。这就是一个 Rust 感知部署工程师的核心日常。

## ONNX：模型的"交换格式"

**ONNX（Open Neural Network Exchange）**是把训练和部署解耦的关键。它是一个开放标准，描述一个神经网络的**计算图**——有哪些算子（卷积、激活、矩阵乘…）、怎么连、权重是多少。你可以把它类比成神经网络界的 PDF：不管用什么软件（PyTorch/TensorFlow）"写"的，导出成 ONNX 后，任何支持 ONNX 的引擎都能"读"和"跑"。

从 PyTorch 导出一行搞定（这是算法同事做的，给你感受一下）：

```python
# Python 侧，仅供理解，不是你写的
torch.onnx.export(model, dummy_input, "detector.onnx",
                  input_names=["images"], output_names=["output"],
                  opset_version=12)
```

导出后你可以用 [Netron](https://netron.app) 这个工具**可视化**这个 `.onnx`——强烈建议每次拿到新模型都先用 Netron 打开看一眼：输入张量叫什么名字、什么形状、输出是什么形状。这些信息决定了你 Rust 代码里怎么喂、怎么解析。**不看 Netron 就写部署代码，等于闭着眼睛穿针。**

> **面试题**：为什么要 ONNX，直接用 PyTorch 的 C++ 版（libtorch）部署不行吗？
> **答**：也行，但 ONNX 更解耦、更轻、跨引擎。libtorch 把你绑死在 PyTorch 运行时上，包很大。ONNX 是中立格式，能被 onnxruntime、TensorRT、各种边缘芯片的推理引擎消费，还能做图优化（算子融合、常量折叠）。工业界部署链路里，"PyTorch → ONNX →（可选 TensorRT）→ 上车"是最主流的一条路。

## Rust 三大推理方案对比

Rust 里跑神经网络，主要有三个选择。理解它们的定位，比记 API 重要。

### `ort`：onnxruntime 的 Rust 绑定 —— 生产首选

`ort` 是微软 [ONNX Runtime](https://onnxruntime.ai)（一个 C++ 写的、工业级、高度优化的推理引擎）的 Rust 绑定。它不是纯 Rust——底层调用 onnxruntime 的动态库（`libonnxruntime.so`），Rust 只是安全封装。

- **优点**：**性能最强、算子最全、后端最多**（CPU、CUDA、TensorRT、DirectML、CoreML…）。onnxruntime 是微软和业界长期打磨的引擎，你能想到的算子它基本都支持，还能自动做图优化。
- **缺点**：依赖一个 C++ 库（需要链接/分发 `.so`），不是纯 Rust，交叉编译到某些嵌入式平台会稍麻烦。
- **结论**：**生产环境部署，尤其要 GPU/TensorRT 加速时，首选 `ort`。** 本章的完整例子就用它。

### `tract`：纯 Rust 推理引擎

`tract` 是 Sonos 公司开源的**纯 Rust**神经网络推理库，支持 ONNX 和 TensorFlow 格式。

- **优点**：**纯 Rust、零 C 依赖**，交叉编译、嵌入式部署非常干净——`cargo build` 出来就是个自包含的东西，没有 `.so` 分发的烦恼。适合小模型、边缘设备、对二进制自包含有要求的场景。
- **缺点**：**只有 CPU**（没有 GPU 后端），算子覆盖不如 onnxruntime 全，遇到新奇算子可能不支持；大模型上性能不如 `ort`。
- **结论**：嵌入式、小模型、纯 Rust 洁癖、CPU 够用时，`tract` 很香。Sonos 自己拿它在音箱上跑唤醒词模型，就是典型场景。

### `candle`：Rust 原生、类 PyTorch

`candle` 是 Hugging Face 出的**Rust 原生**深度学习框架，API 风格贴近 PyTorch，支持 CPU 和 CUDA，能加载多种格式（含 safetensors），甚至能**训练**。

- **优点**：不只是推理，是个完整的张量计算框架；对 LLM/Transformer 支持好（Hugging Face 的主场）；Rust 原生、API 现代。
- **缺点**：相对年轻，生态和算子稳定性仍在快速演进；用它跑传统检测/分割模型，不如直接 ONNX 那条路成熟。
- **结论**：如果你要在 Rust 里做更"重"的事——自定义模型结构、跑大模型、甚至微调——`candle` 是最有前途的选择。传统感知模型部署，仍推荐走 ONNX + `ort`。

一句话选型：

| 场景 | 选谁 |
|---|---|
| 生产部署、要 GPU/TensorRT、算子全 | **`ort`** |
| 嵌入式/边缘、纯 Rust、CPU 小模型 | **`tract`** |
| Rust 原生框架、LLM/Transformer、想训练 | **`candle`** |

> **真实项目里**：你工作环境里的 `inf/` 就是"Rust 做大规模推理流水线"的落地实例。它把一套原本 Python 的 `pplinf` 视频/算法推理流水线移植成 Rust，负责**离线批量推理与评测**——按 task 组织 testcase、给每个视频/图像 handle 配置模型部署路径（`model_deployment_path: "./models"`）和数据类型（`video`/`image`），批量调度到执行器上跑推理二进制（如 `sdk_complete_sample`），再把结果转成统一的 cvproto 格式供评测。注意它的分工：**Rust 负责的是流水线编排、批处理、资源采集、结果落盘与格式转换**，真正的模型前向计算在被调度的推理二进制里。这恰好印证了本章的分工哲学——Rust 极擅长做**可靠的推理基础设施**，把成千上万个视频稳定、可复现地喂给模型跑完。数据闭环里，这种离线推理/评测流水线的吞吐和稳定性，直接决定模型迭代速度。

## 用 `ort` 跑一个 ONNX 检测模型：完整骨架

现在上硬菜。下面是一个**结构完整、接近可运行**的 `ort` 推理骨架：加载模型 → 前处理（复用 4.2 的 letterbox 和 to_nchw）→ `run` → 解析输出。以一个 YOLO 风格的检测网络为例（输入 `[1,3,640,640]`，输出 `[1, N, 85]`：N 个候选框，每个 85 维 = 4 坐标 + 1 目标置信度 + 80 类分数）。

```toml
# Cargo.toml
[dependencies]
ort = "2"          # onnxruntime 绑定
ndarray = "0.16"
image = "0.25"
anyhow = "1"
```

```rust,ignore
use anyhow::Result;
use ndarray::{Array4, Axis};
use ort::session::{Session, builder::GraphOptimizationLevel};
use ort::value::Value;

/// 一个检测框，图像像素坐标系，尚未做 NMS。
#[derive(Debug, Clone, Copy)]
pub struct Detection {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    pub score: f32,   // 目标置信度 × 类别分数
    pub class: usize, // 类别 id
}

pub struct Detector {
    session: Session,
    in_w: u32,
    in_h: u32,
    conf_thresh: f32,
}

impl Detector {
    /// 加载 .onnx 模型，构建会话。生产里这一步只做一次，Detector 复用。
    pub fn new(onnx_path: &str) -> Result<Self> {
        let session = Session::builder()?
            // 图优化：算子融合、常量折叠，onnxruntime 自动做
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(4)?
            // 要 GPU 就在这里注册 execution provider，见下文
            .commit_from_file(onnx_path)?;
        Ok(Self { session, in_w: 640, in_h: 640, conf_thresh: 0.25 })
    }

    /// 对一张已经前处理好的 NCHW 张量跑推理，返回原始检测（letterbox 坐标系）。
    pub fn infer(&mut self, input: Array4<f32>) -> Result<Vec<Detection>> {
        // 1) 把 ndarray 张量包成 ort 的输入 Value
        let input_value = Value::from_array(input)?;

        // 2) run！输入名要和模型里的一致（用 Netron 查，这里假设是 "images"）
        let outputs = self.session.run(ort::inputs!["images" => input_value]?)?;

        // 3) 取出输出张量。输出名假设是 "output"，形状 [1, N, 85]
        let output = outputs["output"].try_extract_tensor::<f32>()?;
        let view = output.view();
        // 去掉 batch 维，拿到 [N, 85]
        let preds = view.index_axis(Axis(0), 0);
        let (n, dim) = (preds.shape()[0], preds.shape()[1]);
        let num_classes = dim - 5;

        // 4) 逐候选框解析：置信度过滤 + 取最高分类别
        let mut dets = Vec::new();
        for i in 0..n {
            let row = preds.index_axis(Axis(0), i);
            let obj = row[4]; // 目标置信度
            if obj < self.conf_thresh {
                continue;
            }
            // 找最高分的类别
            let (mut best_c, mut best_s) = (0usize, 0.0f32);
            for c in 0..num_classes {
                let s = row[5 + c];
                if s > best_s {
                    best_s = s;
                    best_c = c;
                }
            }
            let score = obj * best_s;
            if score < self.conf_thresh {
                continue;
            }
            // YOLO 输出是中心点 + 宽高 (cx, cy, w, h)，转成角点
            let (cx, cy, w, h) = (row[0], row[1], row[2], row[3]);
            dets.push(Detection {
                x1: cx - w / 2.0,
                y1: cy - h / 2.0,
                x2: cx + w / 2.0,
                y2: cy + h / 2.0,
                score,
                class: best_c,
            });
        }
        Ok(dets)
    }
}
```

把它和 4.2 的前处理、以及下面的 NMS 串起来，一帧的完整链路是：

```rust,ignore
use anyhow::Result;

fn detect_frame(det: &mut Detector, img_path: &str) -> Result<Vec<Detection>> {
    // ---- 前处理（4.2 章）----
    let img = image::open(img_path)?.to_rgb8();
    let lb = letterbox(&img, det.in_w, det.in_h);     // 等比缩放 + 填充
    let p = Preproc { in_w: det.in_w, in_h: det.in_h,
                      mean: [0.0; 3], std: [1.0; 3], bgr: false }; // YOLO 常只 /255
    let tensor = to_nchw(&lb.image, &p);              // [1,3,640,640]

    // ---- 推理 ----
    let raw = det.infer(tensor)?;                     // letterbox 坐标系的框

    // ---- 后处理 ----
    let kept = nms(raw, 0.45);                        // 非极大值抑制，见下节
    // 把框从 letterbox 坐标还原到原图（4.2 的 scale_box_back）
    let (ow, oh) = (img.width(), img.height());
    let final_dets = kept.into_iter().map(|mut d| {
        let (x1, y1, x2, y2) =
            scale_box_back((d.x1, d.y1, d.x2, d.y2), &lb, ow, oh);
        d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;
        d
    }).collect();
    Ok(final_dets)
}
```

**注意这个结构的工程含义**：`Detector::new` 只调一次（加载模型是重活，几百毫秒到几秒），`infer` 每帧调用。真实系统里 `Detector` 是个长期存活的对象，被感知线程持有。绝对不要每帧 `new` 一个——那是新手最容易犯的、能让帧率掉到个位数的错。

## 后处理：NMS 非极大值抑制

检测网络对**同一个物体会吐出好多个高度重叠的框**（一辆车周围可能有十几个框，都说"这是车"）。**NMS（Non-Maximum Suppression，非极大值抑制）**就是把这堆重叠框收拾成一个的经典算法。它几乎是每个检测模型后处理的标配，也是常见面试手写题。

**算法逻辑**（贪心）：

1. 把所有框按置信度**从高到低排序**。
2. 取出当前分最高的框，收进结果，作为"胜者"。
3. 计算所有剩余框和这个胜者的 **IoU**，把 IoU 超过阈值（比如 0.45）的都**抑制掉**（它们被认为是同一个物体的重复框）。
4. 在剩下的框里重复 2~3，直到没框了。

用数学说，两个框 A、B 的 IoU 就是 4.1 节那个交并比。NMS 的核心就是"分最高的框留下，和它太像的都删掉"。

Rust 实现，包含 IoU 计算：

```rust,ignore
/// 两个框的 IoU（交并比）。框用 (x1,y1,x2,y2) 表示。
fn iou(a: &Detection, b: &Detection) -> f32 {
    // 交集矩形
    let ix1 = a.x1.max(b.x1);
    let iy1 = a.y1.max(b.y1);
    let ix2 = a.x2.min(b.x2);
    let iy2 = a.y2.min(b.y2);
    let iw = (ix2 - ix1).max(0.0);
    let ih = (iy2 - iy1).max(0.0);
    let inter = iw * ih;

    let area_a = (a.x2 - a.x1).max(0.0) * (a.y2 - a.y1).max(0.0);
    let area_b = (b.x2 - b.x1).max(0.0) * (b.y2 - b.y1).max(0.0);
    let union = area_a + area_b - inter;
    if union <= 0.0 { 0.0 } else { inter / union }
}

/// 非极大值抑制。按类别分别做（不同类的框不互相抑制）。
pub fn nms(mut dets: Vec<Detection>, iou_thresh: f32) -> Vec<Detection> {
    // 按 score 从高到低排序
    dets.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());

    let mut keep: Vec<Detection> = Vec::new();
    let mut suppressed = vec![false; dets.len()];

    for i in 0..dets.len() {
        if suppressed[i] {
            continue;
        }
        keep.push(dets[i]);
        // 抑制后面所有与 i 同类、且 IoU 过高的框
        for j in (i + 1)..dets.len() {
            if suppressed[j] {
                continue;
            }
            if dets[j].class == dets[i].class && iou(&dets[i], &dets[j]) > iou_thresh {
                suppressed[j] = true;
            }
        }
    }
    keep
}
```

几个真实项目里的细节：

- **按类别分开做**：一辆车的框和一个行人的框就算重叠也不该互相抑制。上面用 `dets[j].class == dets[i].class` 实现。
- **复杂度**：朴素 NMS 是 O(n²)。框极多时（几万个候选）会成为瓶颈，工程上会先按置信度粗筛（`conf_thresh`）把候选砍到几百个再做，或者上更快的变体。
- **Soft-NMS**：硬删太粗暴——两辆真正挨得很近的车，其中一辆可能被误删。Soft-NMS 不删框，而是**按 IoU 衰减它的分数**，缓解密集场景漏检。拥堵路口的车流是它的典型用武之地。
- **阈值是权衡**：`iou_thresh` 调高→保留更多框→密集场景召回好但可能有重复框；调低→抑制更狠→干净但可能删掉挨得近的真目标。没有万能值，按场景标定。

> **中高级视角**：NMS 是"检测器输出"和"世界模型"之间的最后一道几何清洗。但它是**无记忆、单帧**的——它不知道上一帧有什么。所以哪怕 NMS 完美，你得到的仍是"这一帧的框"，没有 ID、没有速度、抖动明显。把单帧检测变成稳定、带 ID、带速度的 `Obstacle`，是 4.5 节跟踪要干的活。检测 + NMS 给"哪里有什么"，跟踪给"它是谁、往哪走"。

## 从 Detection 到 Obstacle

别忘了我们的终极目标是 1.3 节的 `Obstacle`。检测给的是**图像 2D 框**，而 `Obstacle` 要**3D 位置、速度**。这中间还差两步：

1. **2D→3D**：靠 4.1 节讲的地平面反投影，或直接用激光雷达/3D 检测网络拿到 3D 框。纯视觉方案在这里估深度，误差随距离放大。
2. **单帧→时序**：靠 4.5 节的跟踪补上 `id` 和 `velocity`。

所以从检测到 `Obstacle` 的骨架大致是：

```rust,ignore
// 把一个 2D 检测 + 估计的深度，粗略填成 Obstacle（简化演示）
fn detection_to_obstacle(d: &Detection, depth_m: f64) -> Obstacle {
    Obstacle {
        id: 0,                       // 待跟踪模块赋值（4.5）
        class: map_class(d.class),   // coco id -> ObstacleClass
        position: [depth_m, 0.0, 0.0], // 真实实现要做反投影，这里占位
        velocity: [0.0; 3],          // 待跟踪模块估计（4.5）
        size: [0.0; 3],
        heading: 0.0,
        confidence: d.score,
    }
}
```

`id` 和 `velocity` 现在是零——它们不是检测能给的，必须等跟踪。这也再次印证 4.1 节的四动词：**检测出框、融合定 3D、跟踪给 ID 和速度**，缺一不可。

## GPU、TensorRT 与量化：让它跑得动

上面的代码在 CPU 上能跑，但车上要实时（几毫秒一帧），CPU 往往不够。三个提速方向你要知道概念：

- **GPU 加速**：`ort` 里注册 CUDA execution provider，推理就跑在 NVIDIA GPU 上，通常快一个数量级。代码上就是在 `Session::builder()` 后 `.with_execution_providers([CUDAExecutionProvider::default().build()])`。前提是环境装了 CUDA 和对应的 onnxruntime GPU 版。
- **TensorRT**：NVIDIA 的推理优化引擎，把 ONNX 图进一步针对具体 GPU 做算子融合、内核选择、精度校准，是车载 NVIDIA 平台（Orin 等）上榨性能的终极手段。`ort` 也支持 TensorRT provider。代价是首次构建 engine 慢、且和具体硬件绑定。
- **量化（quantization）**：把模型权重和计算从 `f32`（32 位浮点）降到 `int8`（8 位整数）。**模型体积缩到 1/4、推理快好几倍、功耗大降**，代价是**精度略降**（需要用校准数据集做"量化感知"或"训练后量化"来把精度损失压到可接受）。边缘/车载部署几乎必用量化——`f32` 精度往往是奢侈品。

> **面试题**：int8 量化为什么能加速，代价是什么？
> **答**：加速来自三方面——数据搬运量减到 1/4（内存带宽是瓶颈时收益巨大）、int8 计算单元吞吐远高于 f32、缓存能装下更多。代价是精度损失：把连续的浮点范围塞进 256 个整数级别，必然有量化误差。工程上用校准数据统计每层激活的数值范围来选量化尺度，把误差控制在可接受范围（检测 mAP 通常掉 0.5~2 个点）。安全攸关的层有时保留高精度（混合精度）。这套权衡是边缘部署的核心手艺。

## 小结

- 行业标准分工：**训练在 Python（PyTorch 生态），部署在 Rust/C++（实时、可靠、无 Python 运行时）**；两者用 **ONNX** 交换格式黏合。拿到新模型先用 Netron 看输入输出。
- Rust 三大推理方案：**`ort`**（onnxruntime 绑定，性能最强、后端最全，生产首选）、**`tract`**（纯 Rust、CPU、嵌入式友好）、**`candle`**（Rust 原生、类 PyTorch、适合 LLM 和想训练的场景）。
- 我们写了 `ort` 加载 ONNX、`run`、解析 `[1,N,85]` 输出的完整骨架；`Detector` 只 `new` 一次、每帧 `infer`。
- **NMS** 用贪心 + IoU 把重叠框收拾成一个，我们从头实现了 `iou` 和 `nms`；工程上注意按类别分开、O(n²) 瓶颈、Soft-NMS、阈值权衡。
- 检测只给 2D 框，要变成 `Obstacle` 还差 **2D→3D**（融合/3D 检测）和**单帧→时序**（跟踪给 ID 和速度）。
- 提速三板斧：**GPU、TensorRT、int8 量化**；量化以少量精度换数倍速度和 1/4 体积，是边缘部署标配。
- `inf/` 是"Rust 做大规模离线推理/评测流水线"的真实落地——Rust 擅长的正是这种可靠的推理基础设施。

下一章我们离开图像，进入 3D 世界——点云。激光雷达给的十几万个点怎么下采样、怎么分出地面、怎么聚成一个个障碍物，全是有数学、有代码的硬算法。
