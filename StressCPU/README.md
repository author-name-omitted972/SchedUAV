# StressCPU

A tiny Linux load generator for controlled CPU interference experiments. It spawns worker threads that busy-wait for a configurable fraction of each period and then sleep until the next period. It is designed for real-time scheduling studies, system stress tests, and HIL simulation where you want repeatable, parametric CPU pressure.&#x20;

---

## Features

* **Periodic busy-wait with duty cycle:** each worker burns CPU for `duty × period`, then idles until the next window.&#x20;
* **Optional SCHED\_DEADLINE workers:** each worker attempts to switch to `SCHED_DEADLINE` with `runtime = deadline = period` and continues even if the change fails. This requires elevated privileges.&#x20;
* **High-priority coordinator:** the main thread tries to raise itself to `SCHED_FIFO` with priority 99 to manage setup and teardown.&#x20;
* **CPU affinity control:** bind threads to specific cores, assigned in round-robin order from a comma-separated list like `4,5,6,7`.&#x20;
* **Phase staggering:** optional randomized start offset per thread to avoid lock-step bursts. Enabled by default.&#x20;
* **Graceful stop:** responds to `SIGINT` and `SIGTERM`.&#x20;
* **Clear console summary:** prints period, staggering state, selected CPUs, per-group loads, total thread count, and an approximate aggregate CPU usage estimate. Messages are in Chinese.&#x20;

---

## How it works

Each worker computes a busy-time `busy_time = duty × period`, then:

1. Actively spins until `t0 + busy_time` using a simple floating-point operation inside the loop to avoid dead-code elimination.
2. Sleeps until `t0 + period` using `sleep_until`, then repeats.
3. Optional randomized phase offset before the first cycle when staggering is enabled.&#x20;

Workers try to adopt `SCHED_DEADLINE` and the main thread tries `SCHED_FIFO 99`. If either attempt fails, a warning is printed and execution continues with the current policy.&#x20;

---

## Build

```bash
# Dependencies: Linux, g++ with C++17, pthreads
g++ -O2 -std=c++17 -pthread StressCPU.cpp -o StressCPU
```

The source includes a small project header `StressCPU.hpp` that defines the CLI config structure. Place it next to the `.cpp` file or adjust include paths as needed.&#x20;

> Note: The program uses Linux-specific headers and syscalls such as `sched_setattr`, CPU affinity APIs, and real-time policies. It is not portable to non-Linux platforms.&#x20;

---

## Usage

```
StressCPU [--period ms] [--duration sec] [--no-stagger] [--cpu c0,c1,...]
          <duty1> <count1> [<duty2> <count2> ...]
```

* `--period ms`
  Period for all workers in milliseconds. Duty is applied relative to this period.&#x20;
* `--duration sec`
  Run for a fixed number of seconds. Without this flag the program runs until Ctrl+C.&#x20;
* `--no-stagger`
  Disable randomized initial phase for each thread. Staggering is enabled by default.&#x20;
* `--cpu c0,c1,...`
  Bind threads to the given CPU list. Threads are assigned round-robin over the list. Example: `--cpu 4,5,6,7`.&#x20;
* `<duty> <count>` pairs
  Create `count` threads with the given duty cycle in `[0.0, 1.0]`. You may provide multiple pairs.&#x20;

If arguments are missing or invalid, the program prints a Chinese usage guide and exits.&#x20;

---

## Examples

```bash
# 5 threads at 10% duty, default period
./StressCPU 0.1 5

# Mix two groups: 5 threads at 10%, 2 threads at 50%, with a 50 ms period
./StressCPU --period 50 0.1 5 0.5 2

# Run for 30 seconds with a 100 ms period, 8 threads at 20%
./StressCPU --duration 30 --period 100 0.2 8

# Pin to CPUs 4–7, 8 threads at 30% with 50 ms period
./StressCPU --cpu 4,5,6,7 --period 50 0.3 8
```

These match the built-in usage examples shown by the program.&#x20;

---

## Permissions and scheduling notes

* Setting `SCHED_FIFO` or `SCHED_DEADLINE` usually requires `CAP_SYS_NICE` or running as root. The program continues if it cannot change policy and will print a warning.&#x20;
* Workers request `SCHED_DEADLINE` with `runtime = deadline = period`. The actual CPU burn time is still governed by the busy-wait duty cycle in the code path, so system-level throttling by CBS is not used here.&#x20;

---

## Output

At startup the program prints a configuration summary, including:

* period in milliseconds and whether staggering is enabled,
* duration setting or “Ctrl+C to stop,”
* selected CPU list if any,
* each `<duty> × threads` group,
* total thread count and an approximate total CPU usage `≈ Σ duty_i × count_i × 100%`.&#x20;

Warnings and affinity results are also printed, for example when CPU binding fails or when a scheduling policy change is not permitted.&#x20;

---
