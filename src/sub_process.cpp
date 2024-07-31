// sub_process.cpp
#include "rviz_drone_control/sub_process.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace rviz_drone_control {

SubProcess::SubProcess(const std::string& cmd) :child_pid(-1), process_pipe(nullptr), command(cmd) {
    command = cmd;
}

SubProcess::~SubProcess() {
    // 析构函数中关闭管道并等待子进程结束
    if (process_pipe) {
        fclose(process_pipe);
    }
    if (child_pid > 0) {
        int status;
        waitpid(child_pid, &status, 0); // 等待子进程结束并获取其状态
    }
}

// 运行子进程并获取其输出
std::string SubProcess::run() {
    // fork一个新进程
    child_pid = fork();
    if (child_pid == -1) {
        // fork失败
        throw std::runtime_error("Failed to fork process");
    } else if (child_pid == 0) {
        // 子进程
        // 替换子进程的执行映像
        execl("/bin/bash", "bash", "-c", command.c_str(), (char *)NULL);
        // 如果execl成功，不会执行到这里
        _exit(EXIT_FAILURE); // execl失败时退出子进程
    } else {
        // 父进程中执行的代码
        // 返回子进程的PID
        return std::to_string(child_pid);
    }
}

// 读取进程的输出
std::string SubProcess::read_output() {
    char buffer[1024];
    std::string output;
    while (fgets(buffer, sizeof(buffer), process_pipe) != nullptr) {
        output += std::string(buffer);
    }
    return output;
}

// 终止子进程（如果需要）
void SubProcess::kill_process() {
    if(child_pid > 0) {
        int status;
        // execl("/bin/bash", "bash", "-c", "pkill -f ", command.c_str(), (char *)NULL);

        if (kill(child_pid, SIGTERM) == -1) {
            // 发送终止信号失败
            perror("Failed to terminate process");
        }
        // int status;
        waitpid(child_pid, &status, 0); // 等待子进程结束并获取其状态
    }
}

} // namespace rviz_drone_control