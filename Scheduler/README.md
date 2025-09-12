# Thread Scheduling Interface

This repository provides a small C++ utility, **ThreadCtl**, and three sample programs that apply Linux scheduling policies to existing threads in a running system. It is designed to help experiment with scheduler settings for complex robotics and UAV workloads where many named threads already exist.

## What this tool does

1. Finds a target thread by its **kernel name** (the content of `/proc/<pid>/task/<tid>/comm`).
2. Moves the thread into a configurable **cpuset** group named `user` under cgroup v1 cpuset. The default file is `/sys/fs/cgroup/cpuset/user/tasks`.
3. Sets the scheduling policy and parameters through the `sched_setattr` syscall.
4. Exposes small helpers to query policy, priority, CPU affinity, and status, and to modify priority and CPU affinity later.

> The tool does not create new threads. It attaches to already running threads whose names you provide.

## Directory layout

- `ThreadCtl.hpp` and `ThreadCtl.cpp` define the interface and implementation.
- `CFS.cpp` shows how to assign **SCHED_OTHER** with a requested `nice` value.
- `RMS.cpp` shows how to assign **SCHED_FIFO** priorities for a rate‑monotonic style setup.
- `EDF.cpp` shows how to assign **SCHED_DEADLINE** with runtime, deadline, and period in **nanoseconds**.

## Build

You can build with any recent GCC or Clang on Linux. No special libraries are required.

```bash
# Build the utility into standalone demos
g++ -O2 -std=gnu++17 -o CFS CFS.cpp ThreadCtl.cpp
g++ -O2 -std=gnu++17 -o RMS RMS.cpp ThreadCtl.cpp
g++ -O2 -std=gnu++17 -o EDF EDF.cpp ThreadCtl.cpp
```

> You may prefer `-static-libstdc++ -static-libgcc` on embedded targets, if available.

## Run requirements

1. Linux with support for `sched_setattr`. Most modern kernels qualify.
2. Sufficient privileges to change scheduling policy and to write to cpuset. Running as `root` is the simplest option.
3. A cpuset cgroup named `user` must exist and be configured. Example setup:

```bash
sudo mkdir -p /sys/fs/cgroup/cpuset/user
echo 0-7 | sudo tee /sys/fs/cgroup/cpuset/user/cpuset.cpus
echo 0   | sudo tee /sys/fs/cgroup/cpuset/user/cpuset.mems
# Optional but common in real-time experiments
echo 1   | sudo tee /sys/fs/cgroup/cpuset/user/cpuset.cpu_exclusive 2>/dev/null || true
```

If your system uses cgroup v2, adapt the cpuset path or mount a v1 cpuset hierarchy for experiments.

## Quick start

The samples expect that your target process has already created threads with specific names, for example PX4 work queues like `wq:vehicle_imu` or application threads like `traj_main`.

### 1) Fair scheduling with SCHED_OTHER

```bash
sudo ./CFS
```

The program looks up each name, moves the thread into the `user` cpuset, and sets policy to `SCHED_OTHER` with the requested nice value. It then prints the process id, thread id, policy, and priority.

### 2) Fixed priority scheduling with SCHED_FIFO

```bash
sudo ./RMS
```

This program assigns fixed priorities. Higher numbers mean higher priority. Use this when you need strict ordering by period in a rate‑monotonic style setup.

### 3) Deadline scheduling with SCHED_DEADLINE

```bash
sudo ./EDF
```

This program sets `(runtime, deadline, period)` per thread in **nanoseconds**. Choose values that match your control loop and sensing periods. Runtime must be less than or equal to period.

## Programmatic use

Here is a minimal example of how to use `ThreadCtl` in your own tool:

```cpp
#include "ThreadCtl.hpp"

int main() {
    // Attach to an existing thread by its exact kernel name
    ThreadCtl ctl("wq:vehicle_imu");

    // Fixed priority example
    ctl.set_policy(SCHED_FIFO, 99);

    // Limit execution to selected CPUs
    ctl.set_cpus({4,5,6,7});

    // Query attributes
    int pol  = ctl.policy();
    int prio = ctl.priority();
    auto cpus = ctl.cpus();
    return 0;
}
```

## API at a glance

```cpp
// Construct by thread name (exact match to /proc/.../comm)
ThreadCtl::ThreadCtl(const std::string& thread_name);

// Read
std::string ThreadCtl::status() const;   // returns the "State:" line from /proc
int         ThreadCtl::policy() const;   // returns numeric policy id
int         ThreadCtl::priority() const; // returns FIFO/RR priority or 0 otherwise
std::vector<int> ThreadCtl::cpus() const;

// Write
void ThreadCtl::set_policy(int policy,
                           int priority = 0,
                           uint64_t runtime_ns = 0,
                           uint64_t deadline_ns = 0,
                           uint64_t period_ns = 0);

void ThreadCtl::set_priority(int prio);
void ThreadCtl::set_cpus(const std::vector<int>& cpus);
```

**Policy parameters**

- `SCHED_OTHER`: use `priority` as the requested nice value in the range `[-20, 19]`.
- `SCHED_FIFO` and `SCHED_RR`: use `priority` as static priority in the range `[1, 99]`.
- `SCHED_DEADLINE`: ignore `priority`. Provide `runtime_ns`, `deadline_ns`, `period_ns` in nanoseconds.

Out‑of‑range inputs are clamped to valid ranges inside the helper.

## Naming and discovery rules

- The tool scans `/proc/<pid>/task/*/comm` and finds the first thread whose name equals the provided string.
- The name comparison is exact. Be sure that your application sets meaningful thread names, for example with `pthread_setname_np`.
- If a name is not found the tool throws an exception. Handle that in your caller or verify that your target is running.

## Common pitfalls

1. The `user` cpuset directory does not exist or is not writable. Create and configure it as shown above.
2. Privilege is insufficient to set real time policy or move tasks between cgroups. Run with proper capabilities.
3. The thread name is incorrect or truncated. Many systems limit names to 15 characters plus the terminator.

## Extending this tool

- Add functions that change `SCHED_DEADLINE` parameters at runtime for adaptive control.
- Expose helpers that park background work on housekeeping cores and leave control loops on isolated cores.
- Build a YAML or JSON driven mapping from thread name to policy for repeatable experiments.

## License

Add your preferred open source license.
