# 2.7 性能工程、unsafe 与 C/C++ FFI

自动驾驶 Rust 工程师迟早会遇到三个现实：一帧点云跑不进预算、模型/驱动只有 C/C++ SDK、优化需要越过安全 Rust 的边界。本章给出可审计的处理方法。

## 先测量：平均值不是实时性

对每个节点至少记录：吞吐量、端到端延迟、排队时间、执行时间，以及 p50/p95/p99/p99.9。平均 8 ms、偶发 80 ms 的控制器，绝不能说“满足 10 ms 周期”。

```rust
use std::time::{Duration, Instant};

fn timed<T>(f: impl FnOnce() -> T) -> (T, Duration) {
    let start = Instant::now();
    let value = f();
    (value, start.elapsed())
}
```

基准测试要固定输入、预热、使用 release 构建，并分开统计冷启动与稳态。先用 flamegraph/perf 找热点，再决定改算法、数据布局、分配策略还是并行度。不要凭感觉优化。

## 预算从端到端倒推

假设感知到控制的 deadline 是 100 ms，可先做一版可核验预算：

| 环节 | 执行预算 | 排队/通信预算 |
|---|---:|---:|
| 感知 | 45 ms | 5 ms |
| 定位与融合 | 10 ms | 3 ms |
| 预测规划 | 20 ms | 4 ms |
| 控制 | 5 ms | 2 ms |
| 余量 | 6 ms | — |

预算必须包含最坏可接受值、超限策略和负责人。模块都“各自达标”但总链路超时，通常是队列、时间戳或调度造成的。

## 数据布局、分配与缓存

优化优先级通常是：减少工作量 → 连续访问 → 减少复制/分配 → 并行 → SIMD。实践中：

- 点云批处理优先 SoA 或紧凑连续数组；热字段与冷字段分离。
- 在循环外 `Vec::with_capacity`，稳态路径复用 buffer；监测容量增长。
- 大帧用所有权转移或 `Arc<[T]>`，但不要把 `Arc` 当成自动零拷贝：序列化、跨进程和 GPU 上传仍可能复制。
- 并行前先看粒度；小任务的调度成本可能高于计算。

## `unsafe` 的正确边界

`unsafe` 不是“关闭检查”，而是作者承担编译器无法证明的安全契约。量产代码遵循：

1. 默认禁止：crate 根设置 `#![deny(unsafe_op_in_unsafe_fn)]`，业务 crate 可再加 `#![forbid(unsafe_code)]`。
2. 集中隔离：仅在小型 `sys`/adapter crate 中出现 `unsafe`。
3. 每个块写 `// SAFETY:`，明确指针有效期、对齐、长度、别名、线程和释放者。
4. 对外只暴露安全 API，并用测试、Miri/消毒器和人工审查验证契约。

```rust
/// # Safety contract
/// `ptr` 在调用期间必须指向 `len` 个已初始化、正确对齐的 f32，且内存不可被并发修改。
unsafe fn sensor_slice<'a>(ptr: *const f32, len: usize) -> &'a [f32] {
    // SAFETY: 调用者承担上述有效性、对齐、初始化和并发契约。
    unsafe { std::slice::from_raw_parts(ptr, len) }
}
```

这里的生命周期不能凭空变安全。更好的封装通常让返回切片借用一个持有 SDK handle 的 Rust 对象，使切片不可能活过底层 buffer。

## C/C++ FFI 检查表

边界类型只用 ABI 稳定表示：`#[repr(C)]` 结构、定宽整数、裸指针和显式长度。不要跨边界直接传 `String`、`Vec`、trait object 或 Rust enum。

```rust
#[repr(C)]
pub struct RawFrame {
    data: *const u8,
    len: usize,
    timestamp_ns: u64,
}
```

接口文档必须回答：谁分配、谁释放；指针能活多久；是否可空；线程安全性；错误码；回调发生在哪个线程；C++ 异常/Rust panic 是否可能穿越 ABI。一般策略是两边都不允许异常展开穿越边界，把失败转换成状态码，RAII wrapper 在 `Drop` 中释放资源。

绑定生成可用 `bindgen`，Rust 暴露 C API 可用 `cbindgen`，C++ 桥接可评估 `cxx`。生成绑定不等于接口安全；安全性来自你写的 wrapper 和契约。

## 性能优化的证据链

一次合格优化提交应包含：固定数据集、优化前后分位数、CPU/内存/分配变化、结果等价性、目标平台信息和回退方案。只给一张“快了 3 倍”的截图，不足以评审。

## 本章验收

选择第 4.4 节的体素下采样：建立 10 万/100 万点基准；输出 p50/p99、峰值内存和每帧分配次数；用 profile 解释前三个热点；做一次优化并用属性测试证明输出不变。另写一个 100 行以内的 FFI wrapper，提交安全契约和故障测试。

## 小结

- 性能结论必须来自目标平台上的分位数和 profile，而不是平均值与直觉。
- `unsafe` 应小、集中、带契约，并被安全抽象包围。
- FFI 的核心是所有权、生命周期、ABI、线程与错误传播契约。

延伸阅读：[Rustonomicon：FFI](https://doc.rust-lang.org/nomicon/ffi.html)、[Cargo profiles](https://doc.rust-lang.org/cargo/reference/profiles.html)。
