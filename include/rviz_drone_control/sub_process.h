// sub_process.h
#ifndef SUB_PROCESS_H
#define SUB_PROCESS_H

#include <cstdio>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <sys/wait.h>
#include <iostream>

namespace rviz_drone_control{
class SubProcess
{
private:
    pid_t child_pid; // 子进程的PID
    FILE* process_pipe; // 用于读取子进程输出的管道
    std::string command; // 要执行的命令

public:
    SubProcess(const std::string& cmd);
    ~SubProcess();

    std::string run();
    std::string read_output();
    void kill_process();

    // 禁止拷贝构造函数和赋值操作符
    SubProcess(const SubProcess&) = delete;
    SubProcess& operator=(const SubProcess&) = delete;

    // 允许移动构造函数和赋值操作符（如果需要的话）
    SubProcess(SubProcess&&) = default;
    SubProcess& operator=(SubProcess&&) = default;
};
} // namespace rviz_drone_contrl

#endif // SUB_PROCESS_H