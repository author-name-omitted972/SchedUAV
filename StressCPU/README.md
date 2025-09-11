
---

# StressCPU

一个使用 **C++14 及以下标准库** 实现的 CPU 占空比“忙等”负载发生器。
根据命令行给定的“占用比例—线程数”成对参数（如 `0.1 5`），创建若干线程，在固定周期内先忙等、再休眠，从而在**不绑核**的前提下，模拟期望的平均 CPU 占用率。

---

## 目录结构

```
StressCPU/
├─ CMakeLists.txt
├─ include/
│  └─ StressCPU.hpp
├─ src/
│  └─ StressCPU.cpp   # 含 main() 与实现
└─ build/             # out-of-source 构建目录
```

---

## 快速开始

### 1) 构建（Linux/macOS，或任何支持 Unix Makefiles 的环境）

```bash
cd StressCPU
mkdir -p build && cd build
cmake .. && make -j
```

生成的可执行文件位于：

```
build/StressCPU
```

> 想要带优化：`cmake -DCMAKE_BUILD_TYPE=Release .. && make -j`

### 2) 运行示例

```bash
# 5 个线程，各自 10% 占空比（默认周期 50ms）
./StressCPU 0.1 5

# 再加 2 个线程各自 50%
./StressCPU 0.1 5 0.5 2

# 更短周期（10ms），占空调制更细、更平滑
./StressCPU --period 10 0.1 5 0.5 2

# 运行 30 秒后自动退出
./StressCPU --duration 30 0.2 8

# 关闭随机相位（可观察更强的同步峰值）
./StressCPU --no-stagger 0.3 4
```

停止：`Ctrl + C`（或使用 `--duration` 指定秒数）。

---

## 命令行

```
StressCPU [--period ms] [--duration sec] [--no-stagger] <duty1> <count1> [<duty2> <count2> ...]
```

| 参数                     | 含义                                                   |
| ---------------------- | ---------------------------------------------------- |
| `--period, -p <ms>`    | 负载调制周期（毫秒）。默认 50 ms。周期越小越平滑，但调度/定时开销相对更高。           |
| `--duration, -d <sec>` | 运行时长（秒）。缺省为无限运行，直到收到信号退出。                            |
| `--no-stagger`         | 关闭随机相位抖动（默认开启）。关闭后多线程负载同相位，瞬时峰值更明显。                  |
| `<duty> <count>`       | 成对参数。`duty` 为占空比（0.0–1.0），`count` 为线程数（正整数）。可重复给出多组。 |

> **总体占用的估算**：未绑核时，进程总 CPU 百分比近似
>
> $$
> \sum_i (\text{duty}_i \times \text{count}_i) \times 100\%
> $$
>
> 例如 `0.1×5 + 0.5×2 = 1.5`，约 **150%**（相当于 1.5 个满核）。

---
