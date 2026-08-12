# 4.2 图像处理基础与 Rust 实现

上一章我们说，视觉感知的最终目的是把图像变成 3D 世界描述，而这条路的第一步、也是每一帧都要重复无数遍的一步，是**把一张图像加工成神经网络能吃的输入张量**。这一步听起来平平无奇，却是新手最容易写错、且写错了不报错只是"精度悄悄掉几个点"的地方。归一化的均值填错、通道顺序 RGB/BGR 搞反、NCHW/NHWC 摆错——模型照样跑，结果照样输出，只是全是垃圾。

这一章我们从零建立图像在内存里的表示，用 Rust 的 `image` 和 `ndarray` 两个 crate，把"读图 → 缩放/裁剪 → 归一化 → 打包成张量"这条**推理前处理（preprocessing）**流水线完整地写出来，并把"检测框坐标怎么映射回原图"这个工程上必踩的坑填平。

## 一张图像在内存里长什么样

先建立最朴素的直觉。一张彩色图像，本质是一个**三维数组**：

- **高度 H（height）**：有多少行像素。
- **宽度 W（width）**：每行多少个像素。
- **通道 C（channel）**：每个像素由几个数字描述。RGB 图是 3 通道（红、绿、蓝），灰度图是 1 通道，带透明度的是 4 通道（RGBA）。

每个数字通常是一个 `u8`（0~255），0 是最暗、255 是最亮。所以一张 `1920×1080` 的 RGB 图，就是 `1920 × 1080 × 3 = 6,220,800` 个 `u8`，约 6 MB——这正是 1.3 节说的"一帧图像约 6 MB"的来历。

在内存里，这些数字通常是**按行、逐像素、逐通道**平铺成一个一维 `Vec<u8>` 的。对 `HWC`（高-宽-通道）排列，像素 `(row, col)` 的第 `c` 个通道，落在下标：

```text
index = (row * W + col) * C + c
```

记住这个公式，你就理解了图像库底层在干什么——所有的"缩放""裁剪"最终都是在算这些下标。

> **中高级视角**：内存布局不是小事。CPU 缓存喜欢**连续访问**。逐行扫图像（沿 `col` 走）是缓存友好的，逐列扫（沿 `row` 跳 `W*C` 个字节）会疯狂 cache miss。当你手写图像算子、发现"逻辑没错但慢十倍"时，十有八九是访问顺序踩了内存布局。点云处理（4.4 节）里这个问题更致命。

## 颜色空间：不止 RGB

RGB 是最常见的，但你会遇到别的，得知道它们为什么存在：

- **RGB / BGR**：同样三个通道，只是顺序反了。**这是头号大坑**：OpenCV 默认 BGR，很多用 OpenCV 训练/导出的模型期望 BGR 输入；而 `image` crate 给你的是 RGB。喂反了模型不报错，精度默默下降。**部署时第一件事就是确认模型要 RGB 还是 BGR。**
- **灰度（grayscale）**：单通道，丢掉颜色只留亮度。车道线检测、某些传统 CV 算法用它，省算力。
- **YUV / YCbCr**：把亮度（Y）和色度（U/V）分开。**相机和视频编码的原生格式几乎都是 YUV**（比如 1.3 节点云那节没提但真实相机常给的 `YUV420`）。为什么？因为人眼对亮度敏感、对色度迟钝，YUV 可以对色度降采样省带宽。你从相机 SDK 或视频解码器拿到的往往是 YUV，要自己转成 RGB 再喂网络。
- **HSV**：色调-饱和度-明度。做"按颜色筛选"（比如找红绿灯的红色区域）时比 RGB 直观。

> **真实项目里**：你工作环境里的 `inf/` 那套离线推理流水线，处理的正是**视频**——它在 `handle` 配置里区分 `data_type: "video"` / `"image"`，视频帧解码出来通常就是 YUV。把 YUV 正确转 RGB、对齐到模型期望的颜色空间，是"离线推理结果能不能对齐线上"的前提之一。颜色空间对不齐，评测出来的指标就是假的。

## 用 `image` crate 读写图像

`image` 是 Rust 生态里最通用的纯 Rust 图像库，读写常见格式、基础变换都有。加依赖：

```toml
# Cargo.toml
[dependencies]
image = "0.25"
ndarray = "0.16"
anyhow = "1"
```

读一张图、看看它的基本信息、再存回去：

```rust,ignore
use anyhow::Result;
use image::{DynamicImage, GenericImageView};

fn load_and_inspect(path: &str) -> Result<DynamicImage> {
    // open 会根据扩展名/魔数自动判断格式并解码
    let img = image::open(path)?;
    let (w, h) = img.dimensions();
    println!("尺寸: {w}x{h}, 颜色类型: {:?}", img.color());
    Ok(img)
}

fn main() -> Result<()> {
    let img = load_and_inspect("frame.jpg")?;

    // 统一转成 RGB8：每像素 3 个 u8，丢掉可能存在的 alpha 通道
    let rgb = img.to_rgb8();

    // 存成 PNG（无损，调试时看着方便）
    rgb.save("out.png")?;
    Ok(())
}
```

`DynamicImage` 是个枚举，能装下灰度/RGB/RGBA/16 位等各种像素类型。实际写推理代码时，**第一步几乎总是 `to_rgb8()`**——把杂七杂八的输入统一成"每像素 3 个 u8、RGB 顺序"的 `RgbImage`，后面的处理就有了确定的假设。

访问单个像素、遍历所有像素：

```rust,ignore
use image::RgbImage;

fn brightness_stats(img: &RgbImage) -> (f64, u8, u8) {
    let mut sum = 0u64;
    let mut min = 255u8;
    let mut max = 0u8;
    // enumerate_pixels 给出 (x, y, &Rgb)，注意 x 是列、y 是行
    for (_x, _y, px) in img.enumerate_pixels() {
        // 粗略亮度：三通道平均
        let lum = ((px[0] as u16 + px[1] as u16 + px[2] as u16) / 3) as u8;
        sum += lum as u64;
        min = min.min(lum);
        max = max.max(lum);
    }
    let n = (img.width() * img.height()) as f64;
    (sum as f64 / n, min, max)
}
```

注意 `image` 用 `(x, y)` 约定：`x` 是列（对应宽 `W`）、`y` 是行（对应高 `H`）。这和数学里习惯的 `(row, col)` 正好反过来，是另一个经典 off-by-混乱源。**心里始终清楚：x↔宽↔col，y↔高↔row。**

## 推理前处理三件套：缩放、裁剪、归一化

神经网络的输入尺寸通常是**固定**的（比如检测网络常见 `640×640`，分类网络 `224×224`）。而相机给的是 `1920×1080`。所以前处理第一步几乎总是缩放。

### 缩放（resize）

```rust,ignore
use image::imageops::FilterType;
use image::RgbImage;

/// 把图像缩放到目标尺寸。FilterType 决定插值方式：
/// - Nearest：最快、最糙（最近邻）
/// - Triangle：双线性，速度/质量平衡，推理前处理常用
/// - CatmullRom / Lanczos3：更平滑但更慢
fn resize_to(img: &RgbImage, w: u32, h: u32) -> RgbImage {
    image::imageops::resize(img, w, h, FilterType::Triangle)
}
```

**这里藏着一个必须知道的坑**：直接 `resize` 到 `640×640` 会**改变长宽比**，把 `1920×1080` 的宽车压成方的，物体形状变形，检测精度会掉。工业界标准做法是 **letterbox（信箱式缩放）**：等比例缩放到能塞进目标框，剩下的边用灰色（常用 114）填充。YOLO 系列全用这套。我们后面讲坐标映射时会把 letterbox 完整实现，因为**它同时决定了坐标怎么映射回原图**。

### 裁剪与 ROI

有时你不想处理整张图，只关心一块**感兴趣区域（ROI, Region of Interest）**——比如已经知道红绿灯大概在图像上部中央，就只裁那一块喂给红绿灯识别网络，省算力也更准。

```rust,ignore
use image::{RgbImage, imageops};

/// 从原图裁出一个 ROI（左上角 x,y，宽 w 高 h）。
/// 会自动裁到图像边界内，避免越界。
fn crop_roi(img: &RgbImage, x: u32, y: u32, w: u32, h: u32) -> RgbImage {
    let x = x.min(img.width().saturating_sub(1));
    let y = y.min(img.height().saturating_sub(1));
    let w = w.min(img.width() - x);
    let h = h.min(img.height() - y);
    imageops::crop_imm(img, x, y, w, h).to_image()
}
```

`crop_imm` 返回的是个借用视图（不拷贝），`.to_image()` 才真正拷出一份独立图像。真实项目里如果只是要读一块区域喂给下一步，能不 `.to_image()` 就别拷——每帧省一次 memcpy，累积起来很可观。

### 归一化（normalization）

这是**最容易错、错了最隐蔽**的一步。像素是 `0~255` 的 `u8`，但神经网络喜欢**小而居中**的浮点输入。归一化把像素线性变换到某个分布。最常见两种约定：

1. **简单缩放到 [0,1]**：`x = px / 255.0`。轻量模型常用。
2. **标准化（standardization）**：先缩到 [0,1]，再按每通道减均值、除标准差：

$$
x_c = \frac{\text{px}_c / 255 - \mu_c}{\sigma_c}
$$

ImageNet 预训练模型几乎都用这套，经典的均值/方差是 `mean = [0.485, 0.456, 0.406]`、`std = [0.229, 0.224, 0.225]`（RGB 顺序）。**这三个数必须和训练时用的完全一致**，否则输入分布和模型期望对不上，精度崩。

> **面试题**：部署一个 PyTorch 训练的检测模型到 Rust，结果精度比 Python 里低很多，可能是哪几个原因？
> **答**：几乎都在前处理。① 颜色通道 RGB/BGR 反了；② 归一化的 mean/std 用错或忘了；③ resize 的插值方式或 letterbox 填充值和训练不一致；④ 通道排布 NCHW/NHWC 搞反；⑤ 像素值该 `/255` 却没除。这五条是"部署精度不一致"的经典嫌疑犯，排查顺序也基本按这个来。模型本身反而很少是问题。

## 用 `ndarray` 表示张量

`Vec<u8>` 能装图像，但一旦要做"每通道减不同的均值""把 HWC 转成 CHW"这类操作，手撸下标既啰嗦又易错。这时候上 **`ndarray`**——Rust 里的多维数组库，相当于 Python 的 NumPy。

一个"张量（tensor）"无非就是带形状的多维数组。神经网络的图像输入张量通常是四维的 `[N, C, H, W]`：

- **N（batch）**：一次喂几张图。离线批量推理时 N 很大，在线单帧 N=1。
- **C（channel）**：通道数，3。
- **H, W**：高、宽。

这个 `NCHW` 排布是 PyTorch、多数 ONNX 模型、TensorRT 的默认。TensorFlow 则常用 `NHWC`（通道在最后）。两者只是同一堆数字的不同摆法，但**摆错了模型直接读到乱码**。

## 完整实现：图像 → NCHW 归一化张量

把上面所有东西串起来，写一个生产可用的前处理函数。这段代码你可以直接抄进项目里。

```rust,ignore
use anyhow::Result;
use image::imageops::FilterType;
use image::RgbImage;
use ndarray::{Array4, ArrayView4};

/// 前处理配置：不同模型的约定不同，用一个 struct 显式带着，避免硬编码。
pub struct Preproc {
    pub in_w: u32,
    pub in_h: u32,
    pub mean: [f32; 3], // 每通道均值（RGB 顺序）
    pub std: [f32; 3],  // 每通道标准差
    pub bgr: bool,      // 模型是否期望 BGR
}

impl Default for Preproc {
    fn default() -> Self {
        // ImageNet 经典参数，NCHW，RGB
        Self {
            in_w: 640,
            in_h: 640,
            mean: [0.485, 0.456, 0.406],
            std: [0.229, 0.224, 0.225],
            bgr: false,
        }
    }
}

/// 把一张 RgbImage 变成 [1, 3, H, W] 的归一化 f32 张量（NCHW）。
/// 注意：这里用简单 resize（会变形），letterbox 版见下一节。
pub fn to_nchw(img: &RgbImage, p: &Preproc) -> Array4<f32> {
    let resized = image::imageops::resize(img, p.in_w, p.in_h, FilterType::Triangle);
    let (w, h) = (p.in_w as usize, p.in_h as usize);

    // 预分配好形状，初值 0
    let mut t = Array4::<f32>::zeros((1, 3, h, w));

    for y in 0..h {
        for x in 0..w {
            let px = resized.get_pixel(x as u32, y as u32);
            // 原始 RGB 三通道
            let mut ch = [px[0] as f32, px[1] as f32, px[2] as f32];
            if p.bgr {
                ch.swap(0, 2); // RGB -> BGR
            }
            for c in 0..3 {
                // 先缩到 [0,1]，再标准化
                let v = (ch[c] / 255.0 - p.mean[c]) / p.std[c];
                // NCHW：t[batch=0, channel=c, row=y, col=x]
                t[[0, c, y, x]] = v;
            }
        }
    }
    t
}

fn main() -> Result<()> {
    let img = image::open("frame.jpg")?.to_rgb8();
    let p = Preproc::default();
    let tensor = to_nchw(&img, &p);
    println!("输入张量形状: {:?}", tensor.shape()); // [1, 3, 640, 640]
    // 这个 tensor 下一章就直接喂给 ort 的 run() 了
    Ok(())
}
```

几个值得强调的工程点：

- **形状先定死**：`Array4::zeros((1, 3, h, w))` 一次性分配，避免在循环里增长。真实系统里这块 buffer 甚至会**复用**（对象池），因为每帧都要一个，反复 malloc/free 是实时系统的大忌。
- **`bgr` 用参数控制而不是硬编码**：模型换了、通道约定变了，改配置不改代码。
- **`mean/std` 是 RGB 顺序的**：如果 `bgr=true`，你要么在 swap 之后仍按物理通道对 mean，要么把 mean 也一起 swap——这里的实现是"swap 通道值、mean 仍按 RGB 索引"，请务必和你的模型训练脚本对齐，这是最容易埋雷的一行。

如果要 `NHWC`，只需把写入下标换成 `t[[0, y, x, c]]` 并把形状建成 `(1, h, w, 3)`。就这么点区别，但摆错就全错。

## Letterbox 与坐标映射回原图

现在填 letterbox 这个坑，它同时解决"不变形缩放"和"框坐标怎么还原"两个问题。

**思路**：以相同比例 `r` 缩放原图，使其正好塞进 `in_w × in_h`，然后把缩放后的图**贴在灰底画布中央**（或左上角），空出来的边填 114。缩放比例和填充偏移必须**记下来**，因为模型输出的框是在 `640×640` 这个 letterbox 图坐标系里的，要减掉偏移、除以比例，才能还原到原图像素坐标。

```rust,ignore
use image::{imageops::FilterType, Rgb, RgbImage};

/// letterbox 的结果：图 + 还原所需的参数
pub struct Letterboxed {
    pub image: RgbImage,
    pub ratio: f32,  // 缩放比例：new = old * ratio
    pub pad_x: f32,  // 左侧填充像素
    pub pad_y: f32,  // 顶部填充像素
}

pub fn letterbox(src: &RgbImage, in_w: u32, in_h: u32) -> Letterboxed {
    let (w0, h0) = (src.width() as f32, src.height() as f32);
    // 取长宽两个方向缩放比的较小者，保证整图塞得进去
    let r = (in_w as f32 / w0).min(in_h as f32 / h0);
    let (nw, nh) = ((w0 * r).round() as u32, (h0 * r).round() as u32);

    let resized = image::imageops::resize(src, nw, nh, FilterType::Triangle);

    // 灰底画布，114 是 YOLO 约定的填充值
    let mut canvas = RgbImage::from_pixel(in_w, in_h, Rgb([114, 114, 114]));
    let pad_x = (in_w - nw) / 2;
    let pad_y = (in_h - nh) / 2;
    image::imageops::overlay(&mut canvas, &resized, pad_x as i64, pad_y as i64);

    Letterboxed {
        image: canvas,
        ratio: r,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
    }
}

/// 把 letterbox 图坐标系下的一个框，还原到原图像素坐标。
/// 输入 (x1,y1,x2,y2) 是模型输出、在 in_w×in_h 图里的坐标。
pub fn scale_box_back(
    b: (f32, f32, f32, f32),
    lb: &Letterboxed,
    orig_w: u32,
    orig_h: u32,
) -> (f32, f32, f32, f32) {
    let unpad = |v: f32, pad: f32| (v - pad) / lb.ratio;
    let x1 = unpad(b.0, lb.pad_x).clamp(0.0, orig_w as f32);
    let y1 = unpad(b.1, lb.pad_y).clamp(0.0, orig_h as f32);
    let x2 = unpad(b.2, lb.pad_x).clamp(0.0, orig_w as f32);
    let y2 = unpad(b.3, lb.pad_y).clamp(0.0, orig_h as f32);
    (x1, y1, x2, y2)
}
```

`scale_box_back` 就是前处理的**逆运算**：先减去填充偏移，再除以缩放比例，最后 `clamp` 到原图范围内（模型偶尔会吐出略微越界的框）。这套"正变换记参数、逆变换还原"的模式，在图像处理里无处不在——**任何你在喂网络前对图像做的几何变换，都必须能把网络输出的坐标反变换回去**，否则你得到的框贴不到原图上。

> **中高级视角**：坐标系是感知工程师的日常噩梦。图像像素坐标、letterbox 坐标、相机坐标、自车坐标、地图坐标……一个障碍物从"图像里第 800 行"变成"自车前方 30 米"，中间要穿过好几层变换（3.2 节讲刚体变换，4.1 节讲过地平面反投影）。养成习惯：**每个坐标值都在脑子里标注它属于哪个坐标系**，就像给物理量标单位一样。搞混坐标系，是感知 bug 的头号来源。

## 卷积与滤波：建立直觉

最后补一点直觉，为下一章的神经网络铺路。你反复听到的"卷积（convolution）"，在图像上其实很朴素：拿一个小矩阵（**卷积核 kernel**，比如 3×3），在图像上滑动，每滑到一处就把核和覆盖的那块像素**对应相乘再求和**，得到输出图对应位置的一个值。

不同的核干不同的事：

- **均值/高斯核**（所有权重为正、和为 1）：**模糊/降噪**，把每个像素替换成邻域加权平均。
- **Sobel 核**（一边正一边负）：**边缘检测**，响应"亮度突变"的地方——这正是传统车道线检测的第一步。

一个 3×3 卷积在像素 `(y,x)` 处的输出：

$$
\text{out}(y,x) = \sum_{i=-1}^{1}\sum_{j=-1}^{1} K(i,j)\cdot \text{img}(y+i, x+j)
$$

手写一个通用 3×3 卷积（灰度图，边界这里简单跳过）：

```rust,ignore
use image::GrayImage;

/// 对灰度图做 3x3 卷积。kernel 按行主序给 9 个权重。
/// 边界像素直接置 0（真实项目会做 padding，这里从简）。
fn conv3x3(img: &GrayImage, kernel: [f32; 9]) -> GrayImage {
    let (w, h) = (img.width(), img.height());
    let mut out = GrayImage::new(w, h);
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let mut acc = 0.0f32;
            for i in 0..3i32 {
                for j in 0..3i32 {
                    let px = img.get_pixel(
                        (x as i32 + j - 1) as u32,
                        (y as i32 + i - 1) as u32,
                    )[0] as f32;
                    acc += kernel[(i * 3 + j) as usize] * px;
                }
            }
            out.put_pixel(x, y, image::Luma([acc.clamp(0.0, 255.0) as u8]));
        }
    }
    out
}

// Sobel 横向边缘核（检测竖直边缘），试试拿它跑车道线：
// [-1, 0, 1,
//  -2, 0, 2,
//  -1, 0, 1]
```

关键洞察：**神经网络里的卷积层，就是把这些核的权重从"人手工设计"变成"从数据里学出来"**。第一层可能自己学出了类似边缘检测的核，越深的层学出越抽象的特征（轮子、车灯、整辆车）。你手写的这个 `conv3x3`，和一个 3×3 卷积层做的算术**完全一样**，只是后者有几百个核、权重是训练来的。理解了这一点，下一章的"深度学习推理"就不再是黑盒——它只是**成千上万次你刚写的这种乘加**，堆在一起。

## 小结

- 图像本质是 `[H, W, C]` 的 `u8` 数组，内存里按 `(row*W+col)*C+c` 平铺；**访问顺序要顺着内存走**，否则慢十倍。
- 颜色空间要认清：**RGB/BGR 顺序反了是头号部署坑**，相机原生常给 YUV，喂网络前要转对。
- 前处理三件套——**缩放、裁剪、归一化**——每一步的参数（插值、mean/std、通道顺序、NCHW/NHWC）**必须和训练时逐字对齐**，否则精度默默崩掉。
- 用 `ndarray` 把图像打包成 `[N,C,H,W]` 张量，我们写了可直接用的 `to_nchw`；下一章它就喂给推理引擎。
- **Letterbox** 等比缩放 + 记录 `ratio/pad`，配一个 `scale_box_back` 逆变换把框还原到原图——"正变换记参数、逆变换还原"是几何处理的通用模式。
- 卷积就是"核在图上滑动做乘加"，神经网络只是把核的权重从手工设计变成从数据学出来。

下一章我们把这个张量真正喂进模型：用 `ort` 加载一个 ONNX 检测网络跑推理，解析输出，再用 NMS 把一堆重叠框收拾干净——完整的 Rust 推理链路。
