/* Linux epoll 接口辅助函数
 * 本文件为 CANopenNode 提供 Linux epoll 接口的辅助函数。epoll 是 Linux 提供的高效 I/O 事件
 * 通知机制，用于监控多个文件描述符的 I/O 事件。本文件封装了 epoll、timerfd 和 eventfd 的
 * 操作，为 CANopenNode 提供基于事件驱动的处理机制。
 */
/**
 * Helper functions for Linux epoll interface to CANopenNode.
 *
 * @file        CO_epoll_interface.h
 * @ingroup     CO_epoll_interface
 * @author      Janez Paternoster
 * @author      Martin Wagner
 * @copyright   2004 - 2020 Janez Paternoster
 * @copyright   2018 - 2020 Neuberger Gebaeudeautomation GmbH
 *
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#ifndef CO_EPOLL_INTERFACE_H
#define CO_EPOLL_INTERFACE_H

#include "CANopen.h"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_socketCAN socketCAN
 * Linux specific interface to CANopenNode.
 *
 * @{
 * Linux includes CAN interface inside its kernel, so called SocketCAN. It operates as a network device. For more
 * information on Linux SocketCAN see https://www.kernel.org/doc/html/latest/networking/can.html
 *
 * Linux specific files for interfacing with Linux SocketCAN are located inside "CANopenNode/socketCAN" directory.
 *
 * CANopenNode runs as a set of non-blocking functions. It can run in single or multiple threads. Best approach for RT
 * IO device can be with two threads:
 * - timer based real-time thread for CAN receive, SYNC and PDO, see   @ref CO_epoll_processRT()
 * - mainline thread for other processing, see @ref CO_epoll_processMain()
 *
 * Main references for Linux functions used here are Linux man pages and the book: The Linux Programming Interface by
 * Michael Kerrisk.
 * @}
 */

/**
 * @defgroup CO_epoll_interface Epoll interface
 * Linux epoll interface to CANopenNode.
 *
 * @ingroup CO_socketCAN
 * @{
 * The Linux epoll API performs a monitoring multiple file descriptors to see if I/O is possible on any of them.
 *
 * CANopenNode uses epoll interface to provide an event based mechanism. Epoll waits for multiple different events, such
 * as: interval timer event, notification event, CAN receive event or socket based event for gateway. CANopenNode
 * non-blocking functions are processed after each event.
 *
 * CANopenNode itself offers functionality for calculation of time, when next interval timer event should trigger the
 * processing. It can also trigger notification events in case of multi-thread operation.
 */

/* epoll、定时器和事件 API 的对象
 * 结构说明：封装了 Linux epoll、timerfd 和 eventfd 的完整状态
 * 成员说明：
 *   - epoll_fd: epoll 文件描述符
 *   - event_fd: 通知事件文件描述符
 *   - timer_fd: 定时器文件描述符
 *   - timerInterval_us: 定时器间隔（微秒），来自 CO_epoll_create()
 *   - timeDifference_us: 自上次 CO_epoll_wait() 执行以来的时间差（微秒）
 *   - timerNext_us: 下一次定时器值（微秒），应用程序可修改以缩短下次 CO_epoll_wait() 的等待时间
 *   - timerEvent: 在 CO_epoll_wait() 内部时为 true，表示定时器事件发生
 *   - previousTime_us: 上次处理调用时的时间值（微秒）
 *   - tm: timerfd 使用的定时器结构体
 *   - ev: epoll_wait 使用的事件结构体
 *   - epoll_new: true 表示需要处理新的 epoll 事件
 */
/**
 * Object for epoll, timer and event API.
 */
typedef struct {
    int epoll_fd;               /**< Epoll file descriptor */
    int event_fd;               /**< Notification event file descriptor */
    int timer_fd;               /**< Interval timer file descriptor */
    uint32_t timerInterval_us;  /**< Interval of the timer in microseconds, from @ref CO_epoll_create() */
    uint32_t timeDifference_us; /**< Time difference since last @ref CO_epoll_wait() execution in microseconds */
    uint32_t timerNext_us;      /**< Timer value in microseconds, which can be changed by application and can shorten
                                   time of next @ref CO_epoll_wait() execution */
    bool_t timerEvent;          /**< True,if timer event is inside @ref CO_epoll_wait() */
    uint64_t previousTime_us;   /**< time value from the last process call in microseconds */
    struct itimerspec tm;       /**< Structure for timerfd */
    struct epoll_event ev;      /**< Structure for epoll_wait */
    bool_t epoll_new;           /**< true, if new epoll event is necessary to process */
} CO_epoll_t;

/* 创建 Linux epoll、timerfd 和 eventfd
 * 函数功能：创建并配置多个 Linux 通知设施，用于触发任务执行。epoll 阻塞并监控多个文件描述符，
 *         timerfd 在恒定的时间间隔触发，eventfd 在外部信号时触发
 * 参数说明：
 *   - ep: 要初始化的 epoll 对象
 *   - timerInterval_us: 定时器间隔（微秒）
 * 返回值说明：
 *   - CO_ERROR_NO: 成功
 *   - CO_ERROR_ILLEGAL_ARGUMENT: 参数非法
 *   - CO_ERROR_SYSCALL: 系统调用失败
 */
/**
 * Create Linux epoll, timerfd and eventfd
 *
 * Create and configure multiple Linux notification facilities, which trigger execution of the task. Epoll blocks and
 * monitors multiple file descriptors, timerfd triggers in constant timer intervals and eventfd triggers on external
 * signal.
 *
 * @param ep This object
 * @param timerInterval_us Timer interval in microseconds
 *
 * @return @ref CO_ReturnError_t CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT or CO_ERROR_SYSCALL.
 */
CO_ReturnError_t CO_epoll_create(CO_epoll_t* ep, uint32_t timerInterval_us);

/* 关闭 epoll、timerfd 和 eventfd
 * 函数功能：关闭并释放 epoll 对象使用的所有文件描述符和资源
 * 参数说明：
 *   - ep: epoll 对象
 * 返回值说明：无返回值
 */
/**
 * Close epoll, timerfd and eventfd
 *
 * @param ep This object
 */
void CO_epoll_close(CO_epoll_t* ep);

/* 等待 epoll 事件
 * 函数功能：阻塞等待在 epoll 上注册的事件：timerfd、eventfd 或应用程序指定的事件。
 *         函数还会计算自上次调用以来的时间差 timeDifference_us，并准备 timerNext_us
 * 参数说明：
 *   - ep: epoll 对象
 * 返回值说明：无返回值
 */
/**
 * Wait for an epoll event
 *
 * This function blocks until event registered on epoll: timerfd, eventfd, or application specified event. Function also
 * calculates timeDifference_us since last call and prepares timerNext_us.
 *
 * @param ep This object
 */
void CO_epoll_wait(CO_epoll_t* ep);

/* epoll 事件的收尾函数
 * 函数功能：必须在 CO_epoll_wait() 之后调用。在两者之间应该执行应用程序指定的处理函数，
 *         这些函数可以检查自己的事件并执行处理。应用程序也可以降低 timerNext_us 变量的值，
 *         如果降低了，定时器将被重新配置，CO_epoll_wait() 将更早被触发
 * 参数说明：
 *   - ep: epoll 对象
 * 返回值说明：无返回值
 */
/**
 * Closing function for an epoll event
 *
 * This function must be called after @ref CO_epoll_wait(). Between them should be application specified processing
 * functions, which can check for own events and do own processing. Application may also lower timerNext_us variable. If
 * lowered, then interval timer will be reconfigured and @ref CO_epoll_wait() will be triggered earlier.
 *
 * @param ep This object
 */
void CO_epoll_processLast(CO_epoll_t* ep);

/* CANopen 复位通信段的函数初始化
 * 函数功能：为 CANopen 对象配置回调函数，在通信复位时调用
 * 参数说明：
 *   - ep: epoll 对象
 *   - co: CANopen 对象
 * 返回值说明：无返回值
 */
/**
 * Initialization of functions in CANopen reset-communication section
 *
 * Configure callbacks for CANopen objects.
 *
 * @param ep This object
 * @param co CANopen object
 */
void CO_epoll_initCANopenMain(CO_epoll_t* ep, CO_t* co);

/* 处理 CANopen 主线函数
 * 函数功能：调用 CO_process() 函数。此函数是非阻塞的，应该周期性执行。
 *         应该在 CO_epoll_wait() 和 CO_epoll_processLast() 函数之间调用
 * 参数说明：
 *   - ep: epoll 对象
 *   - co: CANopen 对象
 *   - enableGateway: 如果为 true，将启用到外部世界的网关
 *   - reset: [输出] 来自 CO_process() 的返回值，指示是否需要复位
 * 返回值说明：无返回值
 */
/**
 * Process CANopen mainline functions
 *
 * This function calls @ref CO_process(). It is non-blocking and should execute cyclically. It should be between @ref
 * CO_epoll_wait() and @ref CO_epoll_processLast() functions.
 *
 * @param ep This object
 * @param co CANopen object
 * @param enableGateway If true, gateway to external world will be enabled.
 * @param [out] reset Return from @ref CO_process().
 */
void CO_epoll_processMain(CO_epoll_t* ep, CO_t* co, bool_t enableGateway, CO_NMT_reset_cmd_t* reset);

/* 处理 CAN 接收和实时函数
 * 函数功能：检查 epoll 的 CAN 接收事件，并处理 CANopen 实时函数：CO_process_SYNC()、
 *         CO_process_RPDO() 和 CO_process_TPDO()。此函数是非阻塞的，应该周期性执行。
 *         应该在 CO_epoll_wait() 和 CO_epoll_processLast() 函数之间调用
 * 使用说明：此函数可以在主线程中使用，也可以在独立的实时线程中使用
 * 注意事项：CANopen 实时函数的处理受 CO_LOCK_OD 保护。另外，必须配置节点 ID 且 CANmodule
 *         必须处于 CANnormal 状态才能进行处理
 * 参数说明：
 *   - ep: epoll 对象指针
 *   - co: CANopen 对象
 *   - realtime: 如果函数从独立的实时线程调用且以短恒定间隔执行，则设置为 true
 * 返回值说明：无返回值
 */
/**
 * Process CAN receive and realtime functions
 *
 * This function checks epoll for CAN receive event and processes CANopen realtime functions: @ref CO_process_SYNC(),
 * @ref CO_process_RPDO() and @ref CO_process_TPDO().  It is non-blocking and should execute cyclically. It should be
 * between @ref CO_epoll_wait() and @ref CO_epoll_processLast() functions.
 *
 * Function can be used in the mainline thread or in own realtime thread.
 *
 * Processing of CANopen realtime functions is protected with @ref CO_LOCK_OD. Also Node-Id must be configured and
 * CANmodule must be in CANnormal for processing.
 *
 * @param ep Pointer to @ref CO_epoll_t object.
 * @param co CANopen object
 * @param realtime Set to true, if function is called from the own realtime thread, and is executed at short constant
 * interval.
 */
void CO_epoll_processRT(CO_epoll_t* ep, CO_t* co, bool_t realtime);

#if ((CO_CONFIG_GTW)&CO_CONFIG_GTW_ASCII) || defined CO_DOXYGEN
/* 网关 ASCII 命令接口类型枚举
 * 枚举说明：定义网关命令接口的类型
 * 枚举值说明：
 *   - CO_COMMAND_IF_DISABLED (-100): 禁用命令接口
 *   - CO_COMMAND_IF_STDIO (-2): 标准输入输出接口
 *   - CO_COMMAND_IF_LOCAL_SOCKET (-1): 本地套接字接口
 *   - CO_COMMAND_IF_TCP_SOCKET_MIN (0): TCP 套接字最小端口号
 *   - CO_COMMAND_IF_TCP_SOCKET_MAX (0xFFFF): TCP 套接字最大端口号
 */
/**
 * Command interface type for gateway-ascii
 */
typedef enum {
    CO_COMMAND_IF_DISABLED = -100,
    CO_COMMAND_IF_STDIO = -2,
    CO_COMMAND_IF_LOCAL_SOCKET = -1,
    CO_COMMAND_IF_TCP_SOCKET_MIN = 0,
    CO_COMMAND_IF_TCP_SOCKET_MAX = 0xFFFF
} CO_commandInterface_t;

/* 网关对象结构体
 * 结构说明：封装网关功能所需的所有状态和配置
 * 成员说明：
 *   - epoll_fd: epoll 文件描述符，来自 CO_epoll_createGtw()
 *   - commandInterface: 命令接口类型或 TCP 端口号，参见 CO_commandInterface_t
 *   - socketTimeout_us: 套接字超时时间（微秒）
 *   - socketTimeoutTmr_us: 套接字超时定时器（微秒）
 *   - localSocketPath: 本地套接字的路径（如果使用本地套接字）
 *   - gtwa_fdSocket: 网关套接字文件描述符
 *   - gtwa_fd: 网关 I/O 流文件描述符
 *   - freshCommand: 新命令指示标志
 */
/**
 * Object for gateway
 */
typedef struct {
    int epoll_fd;                 /**< Epoll file descriptor, from @ref CO_epoll_createGtw() */
    int32_t commandInterface;     /**< Command interface type or tcp port number, see @ref CO_commandInterface_t */
    uint32_t socketTimeout_us;    /**< Socket timeout in microseconds */
    uint32_t socketTimeoutTmr_us; /**< Socket timeout timer in microseconds */
    char* localSocketPath;        /**< Path in case of local socket */
    int gtwa_fdSocket;            /**< Gateway socket file descriptor */
    int gtwa_fd;                  /**< Gateway io stream file descriptor */
    bool_t freshCommand;          /**< Indication of fresh command */
} CO_epoll_gtw_t;

/* 为网关 ASCII 命令接口创建套接字并添加到 epoll
 * 函数功能：根据参数配置标准输入输出接口、本地套接字或 IP 套接字
 * 参数说明：
 *   - epGtw: 要初始化的网关对象
 *   - epoll_fd: 已配置的 epoll 文件描述符
 *   - commandInterface: CO_commandInterface_t 中的命令接口类型
 *   - socketTimeout_ms: 已建立套接字连接的超时时间（毫秒）
 *   - localSocketPath: 文件路径（如果 commandInterface 是本地套接字）
 * 返回值说明：
 *   - CO_ERROR_NO: 成功
 *   - CO_ERROR_ILLEGAL_ARGUMENT: 参数非法
 *   - CO_ERROR_SYSCALL: 系统调用失败
 */
/**
 * Create socket for gateway-ascii command interface and add it to epoll
 *
 * Depending on arguments function configures stdio interface or local socket or IP socket.
 *
 * @param epGtw This object
 * @param epoll_fd Already configured epoll file descriptor
 * @param commandInterface Command interface type from CO_commandInterface_t
 * @param socketTimeout_ms Timeout for established socket connection in [ms]
 * @param localSocketPath File path, if commandInterface is local socket
 *
 * @return @ref CO_ReturnError_t CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT or CO_ERROR_SYSCALL.
 */
CO_ReturnError_t CO_epoll_createGtw(CO_epoll_gtw_t* epGtw, int epoll_fd, int32_t commandInterface,
                                    uint32_t socketTimeout_ms, char* localSocketPath);

/* 关闭网关 ASCII 套接字
 * 函数功能：关闭并释放网关使用的套接字资源
 * 参数说明：
 *   - epGtw: 网关对象
 * 返回值说明：无返回值
 */
/**
 * Close gateway-ascii sockets
 *
 * @param epGtw This object
 */
void CO_epoll_closeGtw(CO_epoll_gtw_t* epGtw);

/* CANopen 复位通信段的网关函数初始化
 * 函数功能：在 CANopen 通信复位时初始化网关相关功能
 * 参数说明：
 *   - epGtw: 网关对象
 *   - co: CANopen 对象
 * 返回值说明：无返回值
 */
/**
 * Initialization of gateway functions  in CANopen reset-communication section
 *
 * @param epGtw This object
 * @param co CANopen object
 */
void CO_epoll_initCANopenGtw(CO_epoll_gtw_t* epGtw, CO_t* co);

/* 处理 CANopen 网关函数
 * 函数功能：检查 epoll 事件并验证套接字连接超时。此函数是非阻塞的，应该周期性执行。
 *         应该在 CO_epoll_wait() 和 CO_epoll_processLast() 函数之间调用
 * 参数说明：
 *   - epGtw: 网关对象
 *   - co: CANopen 对象
 *   - ep: epoll 对象指针
 * 返回值说明：无返回值
 */
/**
 * Process CANopen gateway functions
 *
 * This function checks for epoll events and verifies socket connection timeout. It is non-blocking and should execute
 * cyclically. It should be between @ref CO_epoll_wait() and @ref CO_epoll_processLast() functions.
 *
 * @param epGtw This object
 * @param co CANopen object
 * @param ep Pointer to @ref CO_epoll_t object.
 */
void CO_epoll_processGtw(CO_epoll_gtw_t* epGtw, CO_t* co, CO_epoll_t* ep);
#endif /* (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII */

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_EPOLL_INTERFACE_H */
