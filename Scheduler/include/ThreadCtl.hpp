//=============================================================================
// include/ThreadCtl.hpp
//=============================================================================
#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <utility>

/**
 * @brief 线程控制类
 *
 * 构造函数按线程名称在 *全系统* /proc/<pid>/task/<tid>/comm 中搜索，
 * 找到后记录 {pid, tid} 并把 tid 写入 /sys/fs/cgroup/cpuset/user/tasks。
 *
 * 仅使用 syscall，而非 <sched.h>，支持 SCHED_FIFO / SCHED_RR / SCHED_DEADLINE。
 */
class ThreadCtl {
public:
    explicit ThreadCtl(const std::string& thread_name);
    ~ThreadCtl() = default;

    // 不可复制，可移动
    ThreadCtl(const ThreadCtl&) = delete;
    ThreadCtl& operator=(const ThreadCtl&) = delete;
    ThreadCtl(ThreadCtl&&) noexcept = default;
    ThreadCtl& operator=(ThreadCtl&&) noexcept = default;

    /* ----------------------- 查询接口 ----------------------- */
    pid_t tid() const noexcept { return tid_; }
    pid_t pid() const noexcept { return pid_; }          //!< 线程所属进程 TGID
    std::string status() const;                          //!< /proc/pid/task/tid/status 的 State 行
    int  policy() const;                                 //!< 当前调度策略
    int  priority() const;                               //!< FIFO/RR 的 static priority
    std::vector<int> cpus() const;                       //!< 当前 CPU 亲和列表

    /* ----------------------- 修改接口 ----------------------- */
    /**
     * @brief 设置调度策略
     * @param policy   SCHED_FIFO / SCHED_RR / SCHED_DEADLINE(=6)
     * @param priority FIFO/RR 的优先级 (1–99)；DEADLINE 可忽略
     * @param runtime_ns   runtime (ns)  – 仅 SCHED_DEADLINE
     * @param deadline_ns  deadline (ns) – 仅 SCHED_DEADLINE
     * @param period_ns    period (ns)   – 仅 SCHED_DEADLINE
     *
     * 若 runtime/deadline/period 为 0，则使用默认 10 / 30 / 30 ms。
     */
    void set_policy(int policy,
                    int priority                = 0,
                    uint64_t runtime_ns         = 0,
                    uint64_t deadline_ns        = 0,
                    uint64_t period_ns          = 0);

    void set_priority(int prio);                         //!< 仅 FIFO/RR
    void set_cpus(const std::vector<int>& cpus);         //!< 设置 CPU 亲和

private:
    static std::pair<pid_t,pid_t> find_pid_tid_by_name(const std::string& name);
    static void add_to_user_cpuset(pid_t tid);

    pid_t pid_{-1};  //!< 线程所属进程 TGID
    pid_t tid_{-1};  //!< 调度实体 ID
};
