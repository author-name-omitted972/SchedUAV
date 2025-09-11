//=============================================================================
// src/ThreadCtl.cpp
//=============================================================================
#include "ThreadCtl.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/sched.h>      // 仅为常量枚举，非 <sched.h>
#include <linux/unistd.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <tuple>

namespace {
//--------------------------------------------------------------------------------------------------
// 与内核一致的 sched_attr 结构
//--------------------------------------------------------------------------------------------------
struct sched_attr {
    uint32_t size;           /* 结构体大小 */
    uint32_t sched_policy;   /* 调度策略 */
    uint64_t sched_flags;    /* 额外标志 */
    int32_t  sched_nice;     /* SCHED_OTHER 的 nice 值 */
    uint32_t sched_priority; /* FIFO/RR 的 static priority */
    /* SCHED_DEADLINE 特有字段 */
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

//--------------------------------------------------------------------------------------------------
// syscall 封装
//--------------------------------------------------------------------------------------------------
inline long sys_sched_setattr(pid_t pid, const sched_attr* attr, unsigned int flags) {
    return syscall(SYS_sched_setattr, pid, attr, flags);
}
inline long sys_sched_getattr(pid_t pid, sched_attr* attr, unsigned int size, unsigned int flags) {
    return syscall(SYS_sched_getattr, pid, attr, size, flags);
}
inline long sys_sched_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t* mask) {
    return syscall(SYS_sched_setaffinity, pid, cpusetsize, mask);
}
inline long sys_sched_getaffinity(pid_t pid, size_t cpusetsize, cpu_set_t* mask) {
    return syscall(SYS_sched_getaffinity, pid, cpusetsize, mask);
}

bool is_numeric(const char* s) {
    for (; *s; ++s) if (*s < '0' || *s > '9') return false;
    return true;
}

} // anonymous namespace
/* ================================================================================================ */
/* 私有工具函数                                                                                     */
/* ================================================================================================ */

std::pair<pid_t,pid_t> ThreadCtl::find_pid_tid_by_name(const std::string& target)
{
    DIR* proc_dir = opendir("/proc");
    if (!proc_dir) throw std::runtime_error("无法打开 /proc 目录");

    struct dirent* proc_dent;
    while ((proc_dent = readdir(proc_dir)) != nullptr) {
        if (proc_dent->d_type != DT_DIR || !is_numeric(proc_dent->d_name))
            continue;

        std::string task_path = std::string("/proc/") + proc_dent->d_name + "/task";
        DIR* task_dir = opendir(task_path.c_str());
        if (!task_dir) continue;                   // 可能权限不足

        struct dirent* task_dent;
        while ((task_dent = readdir(task_dir)) != nullptr) {
            if (task_dent->d_type != DT_DIR || !is_numeric(task_dent->d_name))
                continue;

            std::string comm_path = task_path + "/" + task_dent->d_name + "/comm";
            std::ifstream comm(comm_path);
            if (!comm) continue;                  // 无权限读取

            std::string name;
            std::getline(comm, name);
            if (name == target) {
                pid_t pid = static_cast<pid_t>(std::atoi(proc_dent->d_name));
                pid_t tid = static_cast<pid_t>(std::atoi(task_dent->d_name));
                closedir(task_dir);
                closedir(proc_dir);
                return {pid, tid};
            }
        }
        closedir(task_dir);
    }
    closedir(proc_dir);
    throw std::runtime_error("未在系统中找到线程：" + target);
}

void ThreadCtl::add_to_user_cpuset(pid_t tid)
{
    std::ofstream ofs("/sys/fs/cgroup/cpuset/user/tasks");
    if (!ofs) throw std::runtime_error("无法写入 cpuset tasks，需要 root 或 CAP_SYS_ADMIN 权限");
    ofs << tid;
    ofs.flush();
    if (!ofs) throw std::runtime_error("写入 cpuset tasks 失败");
}

/* ================================================================================================ */
/* 构造 / 查询接口                                                                                  */
/* ================================================================================================ */

ThreadCtl::ThreadCtl(const std::string& thread_name)
{
    std::tie(pid_, tid_) = find_pid_tid_by_name(thread_name);
    add_to_user_cpuset(tid_);
}

std::string ThreadCtl::status() const
{
    std::ifstream fs("/proc/" + std::to_string(pid_) +
                     "/task/"  + std::to_string(tid_) +
                     "/status");
    if (!fs) throw std::runtime_error("无法读取线程 status 文件");

    std::string line;
    while (std::getline(fs, line))
        if (line.rfind("State:", 0) == 0)
            return line.substr(6);                // 返回 State: 后面的内容

    return "UNKNOWN";
}

int ThreadCtl::policy() const
{
    sched_attr attr{};
    attr.size = sizeof(attr);
    if (sys_sched_getattr(tid_, &attr, sizeof(attr), 0) < 0)
        throw std::runtime_error("sched_getattr 失败: " + std::string(strerror(errno)));
    return attr.sched_policy;
}

int ThreadCtl::priority() const
{
    sched_attr attr{};
    attr.size = sizeof(attr);
    if (sys_sched_getattr(tid_, &attr, sizeof(attr), 0) < 0)
        throw std::runtime_error("sched_getattr 失败: " + std::string(strerror(errno)));
    return attr.sched_priority;
}

std::vector<int> ThreadCtl::cpus() const
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    if (sys_sched_getaffinity(tid_, sizeof(mask), &mask) < 0)
        throw std::runtime_error("sched_getaffinity 失败: " + std::string(strerror(errno)));

    std::vector<int> res;
    for (int i = 0; i < CPU_SETSIZE; ++i)
        if (CPU_ISSET(i, &mask)) res.push_back(i);
    return res;
}

/* ================================================================================================ */
/* 修改接口                                                                                         */
/* ================================================================================================ */

void ThreadCtl::set_policy(int policy,
                           int priority,
                           uint64_t runtime_ns,
                           uint64_t deadline_ns,
                           uint64_t period_ns)
{
    sched_attr attr{};
    attr.size         = sizeof(attr);
    attr.sched_policy = policy;

    if (policy == SCHED_FIFO || policy == SCHED_RR) {
        int priority_val = priority;
        if (priority_val < 1) priority_val = 1;
        if (priority_val > 99)  priority_val = 99;
        attr.sched_priority = priority_val;
        attr.sched_nice     = 0;
        attr.sched_runtime  = 0;
        attr.sched_deadline = 0;
        attr.sched_period   = 0;
    } else if (policy == SCHED_DEADLINE) {
        attr.sched_runtime  = runtime_ns;
        attr.sched_deadline = deadline_ns;
        attr.sched_period   = period_ns;
        attr.sched_priority = 0;
        attr.sched_nice     = 0;
    } else if (policy == SCHED_OTHER) {
        int nice_val = priority;
        if (nice_val < -20) nice_val = -20;
        if (nice_val > 19)  nice_val = 19;
        attr.sched_nice     = nice_val;
        attr.sched_priority = 0;
        attr.sched_runtime  = 0;
        attr.sched_deadline = 0;
        attr.sched_period   = 0;
    }

    if (sys_sched_setattr(tid_, &attr, 0) < 0)
        throw std::runtime_error("sched_setattr 失败: " + std::string(strerror(errno)));
}

void ThreadCtl::set_priority(int prio)
{
    sched_attr attr{};
    attr.size = sizeof(attr);
    if (sys_sched_getattr(tid_, &attr, sizeof(attr), 0) < 0)
        throw std::runtime_error("sched_getattr 失败: " + std::string(strerror(errno)));

    attr.sched_priority = prio;
    if (sys_sched_setattr(tid_, &attr, 0) < 0)
        throw std::runtime_error("sched_setattr 失败: " + std::string(strerror(errno)));
}

void ThreadCtl::set_cpus(const std::vector<int>& cpus)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    for (int cpu : cpus) CPU_SET(cpu, &mask);
    if (sys_sched_setaffinity(tid_, sizeof(mask), &mask) < 0)
        throw std::runtime_error("sched_setaffinity 失败: " + std::string(strerror(errno)));
}
