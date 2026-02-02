/* Linux epoll 接口辅助函数，用于 CANopenNode 的事件驱动架构实现 */
/*
 * Helper functions for Linux epoll interface to CANopenNode.
 *
 * @file        CO_epoll_interface.c
 * @author      Janez Paternoster
 * @author      Martin Wagner
 * @copyright   2004 - 2015 Janez Paternoster
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

/* 启用 GNU 扩展以支持 accept4() 套接字函数 */
/* following macro is necessary for accept4() function call (sockets) */
#define _GNU_SOURCE

#include "CO_epoll_interface.h"

#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <time.h>
#include <fcntl.h>

#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
#include <stdio.h>
#include <ctype.h>
#include <limits.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <signal.h>

#ifndef LISTEN_BACKLOG
#define LISTEN_BACKLOG 50
#endif
#endif /* (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII */

/* 当 CAN TX 缓冲区满时，重新调用 CANsend() 的延迟时间（微秒） */
/* delay for recall CANsend(), if CAN TX buffer is full */
#ifndef CANSEND_DELAY_US
#define CANSEND_DELAY_US 100
#endif

/* EPOLL 事件处理机制 ********************************************************/
/* EPOLL **********************************************************************/
/* 
 * 函数功能：获取单调时钟时间（微秒）
 * 
 * 执行步骤：
 *   步骤1：调用 clock_gettime() 获取 CLOCK_MONOTONIC 时钟的当前时间
 *   步骤2：将秒转换为微秒并加上纳秒转换为微秒的值
 * 
 * 返回值说明：
 *   返回当前单调时钟时间，单位为微秒（64位无符号整数）
 *   单调时钟不受系统时间调整的影响，适用于计算时间间隔
 */
/* Helper function - get monotonic clock time in microseconds */
static inline uint64_t
clock_gettime_us(void) {
    struct timespec ts;

    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/*
 * 函数功能：创建并配置 epoll 对象（核心函数）
 * 
 * 本函数是 Linux epoll 事件驱动机制的核心初始化函数，用于创建 epoll 实例并配置
 * 事件通知和定时器功能。epoll 是 Linux 提供的高效 I/O 事件通知机制。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：创建 epoll 文件描述符（epoll_fd）作为事件监听器
 *   步骤3：创建 eventfd 用于线程间通知（非阻塞模式）
 *   步骤4：将 eventfd 添加到 epoll 监听列表，监听可读事件（EPOLLIN）
 *   步骤5：创建 timerfd 用于周期性定时器（非阻塞模式）
 *   步骤6：配置定时器间隔和初始触发时间
 *   步骤7：将 timerfd 添加到 epoll 监听列表
 *   步骤8：初始化时间相关变量
 * 
 * 参数说明：
 *   ep - epoll 对象指针，用于存储创建的 epoll 相关文件描述符
 *   timerInterval_us - 定时器间隔时间，单位微秒，决定主循环的执行周期
 * 
 * 返回值说明：
 *   CO_ERROR_NO - 成功创建
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 *   CO_ERROR_SYSCALL - 系统调用失败（epoll_create/eventfd/timerfd_create 等）
 */
CO_ReturnError_t
CO_epoll_create(CO_epoll_t* ep, uint32_t timerInterval_us) {
    int ret;
    struct epoll_event ev = {0};

    if (ep == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* 配置主线程的 epoll 事件监听器 */
    /* Configure epoll for mainline */
    ep->epoll_new = false;
    ep->epoll_fd = epoll_create(1);
    if (ep->epoll_fd < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "epoll_create()");
        return CO_ERROR_SYSCALL;
    }

    /* 创建 eventfd 用于线程间事件通知，并将其添加到 epoll 监听 */
    /* Configure eventfd for notifications and add it to epoll */
    ep->event_fd = eventfd(0, EFD_NONBLOCK);
    if (ep->event_fd < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "eventfd()");
        return CO_ERROR_SYSCALL;
    }
    ev.events = EPOLLIN;
    ev.data.fd = ep->event_fd;
    ret = epoll_ctl(ep->epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);
    if (ret < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(event_fd)");
        return CO_ERROR_SYSCALL;
    }

    /* 创建定时器 fd，配置定时器间隔，并添加到 epoll 监听 */
    /* Configure timer for timerInterval_us and add it to epoll */
    ep->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (ep->timer_fd < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "timerfd_create()");
        return CO_ERROR_SYSCALL;
    }
    ep->tm.it_interval.tv_sec = timerInterval_us / 1000000;
    ep->tm.it_interval.tv_nsec = (timerInterval_us % 1000000) * 1000;
    ep->tm.it_value.tv_sec = 0;
    ep->tm.it_value.tv_nsec = 1;
    ret = timerfd_settime(ep->timer_fd, 0, &ep->tm, NULL);
    if (ret < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "timerfd_settime");
        return CO_ERROR_SYSCALL;
    }
    ev.events = EPOLLIN;
    ev.data.fd = ep->timer_fd;
    ret = epoll_ctl(ep->epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);
    if (ret < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(timer_fd)");
        return CO_ERROR_SYSCALL;
    }
    /* 初始化时间跟踪变量 */
    ep->timerInterval_us = timerInterval_us;
    ep->previousTime_us = clock_gettime_us();
    ep->timeDifference_us = 0;

    return CO_ERROR_NO;
}

/*
 * 函数功能：关闭并清理 epoll 对象
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：关闭 epoll 文件描述符
 *   步骤3：关闭 eventfd 文件描述符
 *   步骤4：关闭 timerfd 文件描述符
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_close(CO_epoll_t* ep) {
    if (ep == NULL) {
        return;
    }

    close(ep->epoll_fd);
    ep->epoll_fd = -1;

    close(ep->event_fd);
    ep->event_fd = -1;

    close(ep->timer_fd);
    ep->timer_fd = -1;
}

/*
 * 函数功能：等待并处理 epoll 事件（核心事件循环函数）
 * 
 * 本函数是事件驱动主循环的核心，它会阻塞等待事件发生（定时器、eventfd 或其他 fd），
 * 然后处理事件并计算时间差。这是实现多线程协调的关键机制之一。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：调用 epoll_wait() 阻塞等待事件（-1 表示无限等待）
 *   步骤3：标记有新事件到达，初始化定时器事件标志
 *   步骤4：计算从上次调用到现在的时间差（用于 CANopen 时间同步）
 *   步骤5：更新上次时间戳
 *   步骤6：设置下次定时器触发间隔的默认值
 *   步骤7：根据事件类型进行处理：
 *          - 中断/信号事件：清除新事件标志
 *          - eventfd 事件：读取事件值（线程间通知机制）
 *          - timerfd 事件：读取定时器值并设置定时器事件标志
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值，事件信息存储在 ep->ev 中，时间差存储在 ep->timeDifference_us 中
 */
void
CO_epoll_wait(CO_epoll_t* ep) {
    if (ep == NULL) {
        return;
    }

    /* 等待事件发生：定时器超时、eventfd 通知或其他 I/O 事件 */
    /* wait for an event */
    int ready = epoll_wait(ep->epoll_fd, &ep->ev, 1, -1);
    ep->epoll_new = true;
    ep->timerEvent = false;

    /* 计算自上次调用以来的时间差，用于 CANopen 协议栈的时间管理 */
    /* calculate time difference since last call */
    uint64_t now = clock_gettime_us();
    ep->timeDifference_us = (uint32_t)(now - ep->previousTime_us);
    ep->previousTime_us = now;
    /* 应用程序可能会降低此值以实现更快的响应 */
    /* application may will lower this */
    ep->timerNext_us = ep->timerInterval_us;

    /* 处理不同类型的事件 */
    /* process event */
    if (ready != 1 && errno == EINTR) {
        /* 来自中断或信号的事件，无需处理，继续 */
        /* event from interrupt or signal, nothing to process, continue */
        ep->epoll_new = false;
    } else if (ready != 1) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "epoll_wait");
        ep->epoll_new = false;
    } else if ((ep->ev.events & EPOLLIN) != 0 && ep->ev.data.fd == ep->event_fd) {
        /* eventfd 可读事件：实时线程通知主线程有数据需要处理 */
        uint64_t val;
        ssize_t s = read(ep->event_fd, &val, sizeof(uint64_t));
        if (s != sizeof(uint64_t)) {
            log_printf(LOG_DEBUG, DBG_ERRNO, "read(event_fd)");
        }
        ep->epoll_new = false;
    } else if ((ep->ev.events & EPOLLIN) != 0 && ep->ev.data.fd == ep->timer_fd) {
        /* timerfd 可读事件：周期性定时器触发，驱动主循环执行 */
        uint64_t val;
        ssize_t s = read(ep->timer_fd, &val, sizeof(uint64_t));
        if (s != sizeof(uint64_t) && errno != EAGAIN) {
            log_printf(LOG_DEBUG, DBG_ERRNO, "read(timer_fd)");
        }
        ep->epoll_new = false;
        ep->timerEvent = true;
    }
}

/*
 * 函数功能：处理事件循环后的清理工作
 * 
 * 本函数在事件处理完成后调用，主要负责处理未识别的事件和调整定时器间隔。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：如果有未处理的事件，记录调试信息
 *   步骤3：如果应用程序要求更短的定时器间隔，则调整定时器
 *   步骤4：将定时器间隔转换为 timespec 结构并设置新的定时器值
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_processLast(CO_epoll_t* ep) {
    if (ep == NULL) {
        return;
    }

    if (ep->epoll_new) {
        log_printf(LOG_DEBUG, DBG_EPOLL_UNKNOWN, ep->ev.events, ep->ev.data.fd);
        ep->epoll_new = false;
    }

    /* 如果应用程序降低了下次定时器间隔，则更新定时器配置 */
    /* lower next timer interval if changed by application */
    if (ep->timerNext_us < ep->timerInterval_us) {
        /* 增加 1 微秒的额外延迟并确保不为零 */
        /* add one microsecond extra delay and make sure it is not zero */
        ep->timerNext_us += 1;
        if (ep->timerInterval_us < 1000000) {
            ep->tm.it_value.tv_nsec = ep->timerNext_us * 1000;
        } else {
            ep->tm.it_value.tv_sec = ep->timerNext_us / 1000000;
            ep->tm.it_value.tv_nsec = (ep->timerNext_us % 1000000) * 1000;
        }
        int ret = timerfd_settime(ep->timer_fd, 0, &ep->tm, NULL);
        if (ret < 0) {
            log_printf(LOG_DEBUG, DBG_ERRNO, "timerfd_settime");
        }
    }
}

/* 主线程处理 ****************************************************************/
/* MAINLINE *******************************************************************/
#ifndef CO_SINGLE_THREAD
/*
 * 函数功能：唤醒主线程的回调函数（多线程协调的关键机制）
 * 
 * 本函数是实现实时线程与主线程之间同步的关键。当实时线程（RT线程）检测到需要
 * 主线程处理的事件时（如 NMT 状态变化、紧急消息等），会调用此回调函数通过
 * eventfd 唤醒阻塞在 epoll_wait() 的主线程。
 * 
 * 执行步骤：
 *   步骤1：将 object 指针转换为 CO_epoll_t 指针
 *   步骤2：向 eventfd 写入值 1，触发 epoll 事件
 *   步骤3：检查写入操作是否成功
 * 
 * 参数说明：
 *   object - epoll 对象指针（void* 类型，需要转换为 CO_epoll_t*）
 * 
 * 返回值说明：
 *   无返回值
 */
/* Send event to wake CO_epoll_processMain() */
static void
wakeupCallback(void* object) {
    CO_epoll_t* ep = (CO_epoll_t*)object;
    uint64_t u = 1;
    ssize_t s;
    s = write(ep->event_fd, &u, sizeof(uint64_t));
    if (s != sizeof(uint64_t)) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "write()");
    }
}
#endif

/*
 * 函数功能：初始化 CANopen 主线程的回调函数
 * 
 * 本函数为 CANopen 协议栈的各个模块注册 wakeupCallback 回调函数，使得这些模块
 * 在需要主线程处理时能够唤醒主线程。这是实现多线程架构的关键配置步骤。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：配置 LSS 从设备的回调函数（如果启用）
 *   步骤3：如果节点 ID 未配置，提前返回
 *   步骤4：为以下 CANopen 模块初始化回调函数：
 *          - NMT (网络管理)
 *          - HBconsumer (心跳消费者)
 *          - EM (紧急消息)
 *          - SDOserver (SDO 服务器)
 *          - SDOclient (SDO 客户端)
 *          - TIME (时间同步)
 *          - LSSmaster (LSS 主设备)
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 *   co - CANopen 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_initCANopenMain(CO_epoll_t* ep, CO_t* co) {
    if (ep == NULL || co == NULL) {
        return;
    }

#ifndef CO_SINGLE_THREAD

    /* 配置 LSS 从设备的回调函数 */
    /* Configure LSS slave callback function */
#if (CO_CONFIG_LSS) & CO_CONFIG_FLAG_CALLBACK_PRE
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE
    CO_LSSslave_initCallbackPre(co->LSSslave, (void*)ep, wakeupCallback);
#endif
#endif

    if (co->nodeIdUnconfigured) {
        return;
    }

    /* 为各个 CANopen 模块配置回调函数，实现事件驱动的多线程协调 */
    /* Configure callback functions */
#if (CO_CONFIG_NMT) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_NMT_initCallbackPre(co->NMT, (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_HB_CONS) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_HBconsumer_initCallbackPre(co->HBcons, (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_EM) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_EM_initCallbackPre(co->em, (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_SDOserver_initCallbackPre(&co->SDOserver[0], (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_SDO_CLI) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_SDOclient_initCallbackPre(&co->SDOclient[0], (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_TIME) & CO_CONFIG_FLAG_CALLBACK_PRE
    CO_TIME_initCallbackPre(co->TIME, (void*)ep, wakeupCallback);
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_FLAG_CALLBACK_PRE
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_MASTER
    CO_LSSmaster_initCallbackPre(co->LSSmaster, (void*)ep, wakeupCallback);
#endif
#endif

#endif /* CO_SINGLE_THREAD */
}

/*
 * 函数功能：处理 CANopen 主线程逻辑（核心处理函数）
 * 
 * 本函数在主线程中被调用，负责处理 CANopen 协议栈的主要逻辑，包括对象字典处理、
 * SDO 服务、网络管理等非实时任务。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：调用 CO_process() 处理 CANopen 对象并获取复位命令
 *   步骤3：检查 CAN 发送缓冲区是否有未发送的消息
 *   步骤4：如果有未发送消息且定时器间隔较长，则缩短定时器间隔以尽快发送
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 *   co - CANopen 对象指针
 *   enableGateway - 是否启用网关功能
 *   reset - 输出参数，返回 NMT 复位命令
 * 
 * 返回值说明：
 *   无返回值，复位命令通过 reset 参数返回
 */
void
CO_epoll_processMain(CO_epoll_t* ep, CO_t* co, bool_t enableGateway, CO_NMT_reset_cmd_t* reset) {
    if (ep == NULL || co == NULL || reset == NULL) {
        return;
    }

    /* 处理 CANopen 对象：对象字典、SDO、网络管理等 */
    /* process CANopen objects */
    *reset = CO_process(co, enableGateway, ep->timeDifference_us, &ep->timerNext_us);

    /* 如果有未发送的 CAN 消息，提前调用 CO_CANmodule_process() */
    /* If there are unsent CAN messages, call CO_CANmodule_process() earlier */
    if (co->CANmodule->CANtxCount > 0 && ep->timerNext_us > CANSEND_DELAY_US) {
        ep->timerNext_us = CANSEND_DELAY_US;
    }
}

/* CAN 接收和实时处理 ********************************************************/
/* CANrx and REALTIME *********************************************************/
/*
 * 函数功能：处理实时任务（核心实时处理函数）
 * 
 * 本函数在实时线程中被调用，负责处理对时间要求严格的 CANopen 任务，包括
 * CAN 消息接收、SYNC 同步、PDO 处理等。这是实时性能的关键函数。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：检查是否有新的 epoll 事件
 *   步骤3：如果有事件，调用 CO_CANrxFromEpoll() 处理 CAN 接收消息
 *   步骤4：在非实时模式或定时器事件触发时，处理 SYNC 和 PDO：
 *          - 锁定对象字典（保护共享数据）
 *          - 处理 SYNC 对象
 *          - 处理接收 PDO (RPDO)
 *          - 处理发送 PDO (TPDO)
 *          - 解锁对象字典
 * 
 * 参数说明：
 *   ep - epoll 对象指针
 *   co - CANopen 对象指针
 *   realtime - 布尔值，指示是否在实时线程中运行
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_processRT(CO_epoll_t* ep, CO_t* co, bool_t realtime) {
    if (co == NULL || ep == NULL) {
        return;
    }

    /* 验证是否有 epoll 事件需要处理（CAN 消息接收事件） */
    /* Verify for epoll events */
    if (ep->epoll_new) {
        if (CO_CANrxFromEpoll(co->CANmodule, &ep->ev, NULL, NULL)) {
            ep->epoll_new = false;
        }
    }

    /* 处理实时任务：SYNC 和 PDO */
    if (!realtime || ep->timerEvent) {
        uint32_t* pTimerNext_us = realtime ? NULL : &ep->timerNext_us;

        /* 锁定对象字典以保护多线程访问 */
        CO_LOCK_OD(co->CANmodule);
        if (!co->nodeIdUnconfigured && co->CANmodule->CANnormal) {
            bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            /* 处理 SYNC 对象，返回是否有 SYNC 事件 */
            syncWas = CO_process_SYNC(co, ep->timeDifference_us, pTimerNext_us);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            /* 处理接收 PDO：从 CAN 总线接收的过程数据对象 */
            CO_process_RPDO(co, syncWas, ep->timeDifference_us, pTimerNext_us);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            /* 处理发送 PDO：向 CAN 总线发送的过程数据对象 */
            CO_process_TPDO(co, syncWas, ep->timeDifference_us, pTimerNext_us);
#endif
            (void)syncWas;
            (void)pTimerNext_us;
        }
        /* 解锁对象字典 */
        CO_UNLOCK_OD(co->CANmodule);
    }
}

/* 网关功能 ******************************************************************/
/* GATEWAY ********************************************************************/
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
/*
 * 函数功能：从网关 ASCII 对象写入响应字符串
 * 
 * 本函数作为回调函数被 CANopen 网关模块调用，用于将响应数据写入到客户端连接。
 * 
 * 执行步骤：
 *   步骤1：将 object 指针转换为文件描述符指针
 *   步骤2：初始化 nWritten 为 count（错误时清空数据）
 *   步骤3：检查文件描述符有效性
 *   步骤4：调用 write() 写入数据到套接字
 *   步骤5：处理写入错误（EAGAIN 表示资源暂时不可用，需重试）
 *   步骤6：如果连接无效，设置 connectionOK 为 0
 * 
 * 参数说明：
 *   object - 文件描述符指针（void* 类型）
 *   buf - 要写入的数据缓冲区
 *   count - 要写入的字节数
 *   connectionOK - 连接状态指针，用于返回连接是否正常
 * 
 * 返回值说明：
 *   返回实际写入的字节数
 */
/* write response string from gateway-ascii object */
static size_t
gtwa_write_response(void* object, const char* buf, size_t count, uint8_t* connectionOK) {
    int* fd = (int*)object;
    /* nWritten = count -> 出错时（文件描述符不存在）数据被清空 */
    /* nWritten = count -> in case of error (non-existing fd) data are purged */
    size_t nWritten = count;

    if (fd != NULL && *fd >= 0) {
        ssize_t n = write(*fd, (const void*)buf, count);
        if (n >= 0) {
            nWritten = (size_t)n;
        } else {
            /* 可能是 EAGAIN - "资源暂时不可用"。需要重试。 */
            /* probably EAGAIN - "Resource temporarily unavailable". Retry. */
            log_printf(LOG_DEBUG, DBG_ERRNO, "write(gtwa_response)");
            nWritten = 0;
        }
    } else {
        *connectionOK = 0;
    }
    return nWritten;
}

/*
 * 函数功能：为 epoll 启用套接字接受功能
 * 
 * 本函数修改 epoll 监听的套接字事件，使用 EPOLLONESHOT 标志确保每次只处理一个连接。
 * 
 * 执行步骤：
 *   步骤1：初始化 epoll_event 结构
 *   步骤2：设置事件类型为 EPOLLIN | EPOLLONESHOT（单次触发）
 *   步骤3：调用 epoll_ctl() 修改 epoll 监听
 * 
 * 参数说明：
 *   epGtw - 网关 epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
static inline void
socketAcceptEnableForEpoll(CO_epoll_gtw_t* epGtw) {
    struct epoll_event ev = {0};
    int ret;

    ev.events = EPOLLIN | EPOLLONESHOT;
    ev.data.fd = epGtw->gtwa_fdSocket;
    ret = epoll_ctl(epGtw->epoll_fd, EPOLL_CTL_MOD, ev.data.fd, &ev);
    if (ret < 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(gtwa_fdSocket)");
    }
}

/*
 * 函数功能：创建网关套接字（核心网关初始化函数）
 * 
 * 本函数创建并配置 CANopen 网关的通信接口，支持三种模式：标准输入输出、
 * 本地 Unix 套接字和 TCP 套接字。网关允许外部应用通过命令行界面访问 CANopen 网络。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：初始化 epGtw 结构体的基本字段
 *   步骤3：根据 commandInterface 类型进行不同的配置：
 *     
 *     A. 标准输入输出模式 (CO_COMMAND_IF_STDIO)：
 *        步骤3.1：设置 gtwa_fd 为标准输入
 *        步骤3.2：记录日志
 *     
 *     B. 本地套接字模式 (CO_COMMAND_IF_LOCAL_SOCKET)：
 *        步骤3.1：创建 AF_UNIX 套接字（非阻塞模式）
 *        步骤3.2：绑定到本地路径
 *        步骤3.3：开始监听连接
 *        步骤3.4：忽略 SIGPIPE 信号（防止连接断开时程序崩溃）
 *     
 *     C. TCP 套接字模式 (CO_COMMAND_IF_TCP_SOCKET_MIN ~ MAX)：
 *        步骤3.1：创建 AF_INET 套接字（非阻塞模式）
 *        步骤3.2：设置 SO_REUSEADDR 选项
 *        步骤3.3：绑定到指定端口
 *        步骤3.4：开始监听连接
 *        步骤3.5：忽略 SIGPIPE 信号
 *   
 *   步骤4：如果 gtwa_fd 有效，将其添加到 epoll 监听
 *   步骤5：如果 gtwa_fdSocket 有效，将其添加到 epoll 监听（EPOLLONESHOT 模式）
 * 
 * 参数说明：
 *   epGtw - 网关 epoll 对象指针
 *   epoll_fd - epoll 文件描述符
 *   commandInterface - 命令接口类型（stdio/本地套接字/TCP套接字端口号）
 *   socketTimeout_ms - 套接字超时时间（毫秒）
 *   localSocketPath - 本地套接字路径（仅用于本地套接字模式）
 * 
 * 返回值说明：
 *   CO_ERROR_NO - 成功创建
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 *   CO_ERROR_SYSCALL - 系统调用失败
 */
CO_ReturnError_t
CO_epoll_createGtw(CO_epoll_gtw_t* epGtw, int epoll_fd, int32_t commandInterface, uint32_t socketTimeout_ms,
                   char* localSocketPath) {
    int ret;
    struct epoll_event ev = {0};

    if (epGtw == NULL || epoll_fd < 0) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    epGtw->epoll_fd = epoll_fd;
    epGtw->commandInterface = commandInterface;

    epGtw->socketTimeout_us = (socketTimeout_ms < (UINT_MAX / 1000 - 1000000)) ? socketTimeout_ms * 1000
                                                                               : (UINT_MAX - 1000000);
    epGtw->gtwa_fdSocket = -1;
    epGtw->gtwa_fd = -1;

    /* 模式 A：标准输入输出接口 */
    if (commandInterface == CO_COMMAND_IF_STDIO) {
        epGtw->gtwa_fd = STDIN_FILENO;
        log_printf(LOG_INFO, DBG_COMMAND_STDIO_INFO);
    } else if (commandInterface == CO_COMMAND_IF_LOCAL_SOCKET) {
        /* 模式 B：本地 Unix 套接字接口 */
        struct sockaddr_un addr;
        epGtw->localSocketPath = localSocketPath;

        /* 创建、绑定并监听本地套接字 */
        /* Create, bind and listen local socket */
        epGtw->gtwa_fdSocket = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (epGtw->gtwa_fdSocket < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "socket(local)");
            return CO_ERROR_SYSCALL;
        }

        memset(&addr, 0, sizeof(struct sockaddr_un));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, localSocketPath, sizeof(addr.sun_path) - 1);
        ret = bind(epGtw->gtwa_fdSocket, (struct sockaddr*)&addr, sizeof(struct sockaddr_un));
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_COMMAND_LOCAL_BIND, localSocketPath);
            return CO_ERROR_SYSCALL;
        }

        ret = listen(epGtw->gtwa_fdSocket, LISTEN_BACKLOG);
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "listen(local)");
            return CO_ERROR_SYSCALL;
        }

        /* 忽略 SIGPIPE 信号，当远程客户端断开连接时可能发生此信号 */
        /* 信号可能由 gtwa_write_response() 函数内的 write 调用触发 */
        /* Ignore the SIGPIPE signal, which may happen, if remote client broke
         * the connection. Signal may be triggered by write call inside
         * gtwa_write_response() function */
        if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
            log_printf(LOG_CRIT, DBG_ERRNO, "signal");
            return CO_ERROR_SYSCALL;
        }

        log_printf(LOG_INFO, DBG_COMMAND_LOCAL_INFO, localSocketPath);
    } else if (commandInterface >= CO_COMMAND_IF_TCP_SOCKET_MIN && commandInterface <= CO_COMMAND_IF_TCP_SOCKET_MAX) {
        /* 模式 C：TCP 套接字接口 */
        struct sockaddr_in addr;
        const int yes = 1;

        /* 创建、绑定并监听 TCP 套接字 */
        /* Create, bind and listen socket */
        epGtw->gtwa_fdSocket = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (epGtw->gtwa_fdSocket < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "socket(tcp)");
            return CO_ERROR_SYSCALL;
        }

        setsockopt(epGtw->gtwa_fdSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

        memset(&addr, 0, sizeof(struct sockaddr_in));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(commandInterface);
        addr.sin_addr.s_addr = INADDR_ANY;

        ret = bind(epGtw->gtwa_fdSocket, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_COMMAND_TCP_BIND, commandInterface);
            return CO_ERROR_SYSCALL;
        }

        ret = listen(epGtw->gtwa_fdSocket, LISTEN_BACKLOG);
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "listen(tcp)");
            return CO_ERROR_SYSCALL;
        }

        /* 忽略 SIGPIPE 信号，当远程客户端断开连接时可能发生此信号 */
        /* 信号可能由 gtwa_write_response() 函数内的 write 调用触发 */
        /* Ignore the SIGPIPE signal, which may happen, if remote client broke
         * the connection. Signal may be triggered by write call inside
         * gtwa_write_response() function */
        if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
            log_printf(LOG_CRIT, DBG_ERRNO, "signal");
            return CO_ERROR_SYSCALL;
        }

        log_printf(LOG_INFO, DBG_COMMAND_TCP_INFO, commandInterface);
    } else {
        epGtw->commandInterface = CO_COMMAND_IF_DISABLED;
    }

    /* 将通信文件描述符添加到 epoll 监听 */
    if (epGtw->gtwa_fd >= 0) {
        ev.events = EPOLLIN;
        ev.data.fd = epGtw->gtwa_fd;
        ret = epoll_ctl(epGtw->epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(gtwa_fd)");
            return CO_ERROR_SYSCALL;
        }
    }
    if (epGtw->gtwa_fdSocket >= 0) {
        /* 准备 epoll 监听新的套接字连接。连接被接受后，将定义用于 I/O 操作的 fd */
        /* prepare epoll for listening for new socket connection. After
         * connection will be accepted, fd for io operation will be defined. */
        ev.events = EPOLLIN | EPOLLONESHOT;
        ev.data.fd = epGtw->gtwa_fdSocket;
        ret = epoll_ctl(epGtw->epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);
        if (ret < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(gtwa_fdSocket)");
            return CO_ERROR_SYSCALL;
        }
    }

    return CO_ERROR_NO;
}

/*
 * 函数功能：关闭网关套接字
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：根据接口类型关闭相应的文件描述符
 *   步骤3：对于本地套接字，从文件系统删除套接字文件
 *   步骤4：重置文件描述符为 -1
 * 
 * 参数说明：
 *   epGtw - 网关 epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_closeGtw(CO_epoll_gtw_t* epGtw) {
    if (epGtw == NULL) {
        return;
    }

    if (epGtw->commandInterface == CO_COMMAND_IF_LOCAL_SOCKET) {
        if (epGtw->gtwa_fd > 0) {
            close(epGtw->gtwa_fd);
        }
        close(epGtw->gtwa_fdSocket);
        /* 从文件系统中删除本地套接字文件 */
        /* Remove local socket file from filesystem. */
        if (remove(epGtw->localSocketPath) < 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "remove(local)");
        }
    } else if (epGtw->commandInterface >= CO_COMMAND_IF_TCP_SOCKET_MIN) {
        if (epGtw->gtwa_fd > 0) {
            close(epGtw->gtwa_fd);
        }
        close(epGtw->gtwa_fdSocket);
    }
    epGtw->gtwa_fd = -1;
    epGtw->gtwa_fdSocket = -1;
}

/*
 * 函数功能：初始化 CANopen 网关
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：初始化网关读取功能，设置响应写入回调函数
 *   步骤3：设置 freshCommand 标志为 true（用于标准输入模式）
 * 
 * 参数说明：
 *   epGtw - 网关 epoll 对象指针
 *   co - CANopen 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_initCANopenGtw(CO_epoll_gtw_t* epGtw, CO_t* co) {
    if (epGtw == NULL || co == NULL || co->nodeIdUnconfigured) {
        return;
    }

    CO_GTWA_initRead(co->gtwa, gtwa_write_response, (void*)&epGtw->gtwa_fd);
    epGtw->freshCommand = true;
}

/*
 * 函数功能：处理网关事件（核心网关处理函数）
 * 
 * 本函数在主循环中被调用，负责处理网关相关的 epoll 事件，包括新连接接受、
 * 命令数据读取、连接超时管理等。这是网关功能的核心事件处理函数。
 * 
 * 执行步骤：
 *   步骤1：参数有效性检查
 *   步骤2：检查是否有网关相关的 epoll 事件
 *   
 *   步骤3：处理套接字接受事件（EPOLLIN on gtwa_fdSocket）：
 *     步骤3.1：调用 accept4() 接受新连接（非阻塞模式）
 *     步骤3.2：如果接受成功，将新连接的 fd 添加到 epoll 监听
 *     步骤3.3：重置套接字超时计时器
 *     步骤3.4：如果失败，重新启用套接字接受
 *   
 *   步骤4：处理数据读取事件（EPOLLIN on gtwa_fd）：
 *     步骤4.1：从套接字读取数据到缓冲区
 *     步骤4.2：根据接口类型处理数据：
 *       
 *       A. 标准输入模式：
 *          步骤4.2.1：简化命令格式，自动添加 "[0] " 前缀（如果缺失）
 *          步骤4.2.2：将数据写入网关对象
 *       
 *       B. 套接字模式（本地或 TCP）：
 *          步骤4.2.1：检查是否收到 EOF（s == 0）
 *          步骤4.2.2：如果收到 EOF，关闭连接并重新启用套接字接受
 *          步骤4.2.3：否则，将数据写入网关对象
 *     
 *     步骤4.3：重置套接字超时计时器
 *   
 *   步骤5：处理错误或挂断事件（EPOLLERR | EPOLLHUP）：
 *     步骤5.1：记录错误日志
 *     步骤5.2：关闭连接
 *   
 *   步骤6：验证套接字超时（针对套接字模式）：
 *     步骤6.1：检查超时计时器是否超过配置的超时时间
 *     步骤6.2：如果超时，关闭当前连接并接受下一个连接
 *     步骤6.3：否则，累加时间差
 * 
 * 参数说明：
 *   epGtw - 网关 epoll 对象指针
 *   co - CANopen 对象指针
 *   ep - epoll 对象指针
 * 
 * 返回值说明：
 *   无返回值
 */
void
CO_epoll_processGtw(CO_epoll_gtw_t* epGtw, CO_t* co, CO_epoll_t* ep) {
    if (epGtw == NULL || co == NULL || ep == NULL) {
        return;
    }

    /* 验证是否有 epoll 事件需要处理 */
    /* Verify for epoll events */
    if (ep->epoll_new && (ep->ev.data.fd == epGtw->gtwa_fdSocket || ep->ev.data.fd == epGtw->gtwa_fd)) {
        /* 事件类型 A：套接字接受事件 - 有新客户端连接 */
        if ((ep->ev.events & EPOLLIN) != 0 && ep->ev.data.fd == epGtw->gtwa_fdSocket) {
            bool_t fail = false;

            /* 接受新连接（非阻塞模式） */
            epGtw->gtwa_fd = accept4(epGtw->gtwa_fdSocket, NULL, NULL, SOCK_NONBLOCK);
            if (epGtw->gtwa_fd < 0) {
                fail = true;
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    log_printf(LOG_CRIT, DBG_ERRNO, "accept(gtwa_fdSocket)");
                }
            } else {
                /* 将新连接的 fd 添加到 epoll 监听 */
                /* add fd to epoll */
                struct epoll_event ev2 = {0};
                ev2.events = EPOLLIN;
                ev2.data.fd = epGtw->gtwa_fd;
                int ret = epoll_ctl(ep->epoll_fd, EPOLL_CTL_ADD, ev2.data.fd, &ev2);
                if (ret < 0) {
                    fail = true;
                    log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(add, gtwa_fd)");
                }
                /* 重置超时计时器 */
                epGtw->socketTimeoutTmr_us = 0;
            }

            if (fail) {
                socketAcceptEnableForEpoll(epGtw);
            }
            ep->epoll_new = false;
        } else if ((ep->ev.events & EPOLLIN) != 0 && ep->ev.data.fd == epGtw->gtwa_fd) {
            /* 事件类型 B：数据读取事件 - 客户端发送了命令数据 */
            char buf[CO_CONFIG_GTWA_COMM_BUF_SIZE];
            size_t space = co->nodeIdUnconfigured ? CO_CONFIG_GTWA_COMM_BUF_SIZE : CO_GTWA_write_getSpace(co->gtwa);

            ssize_t s = read(epGtw->gtwa_fd, buf, space);

            if (space == 0 || co->nodeIdUnconfigured) {
                /* 继续或清空数据 */
                /* continue or purge data */
            } else if (s < 0 && errno != EAGAIN) {
                log_printf(LOG_DEBUG, DBG_ERRNO, "read(gtwa_fd)");
            } else if (s >= 0) {
                if (epGtw->commandInterface == CO_COMMAND_IF_STDIO) {
                    /* 简化标准输入的命令接口，使难以输入的序列变为可选 */
                    /* 如果缺失，则在字符串前添加 "[0] " */
                    /* simplify command interface on stdio, make hard to type
                     * sequence optional, prepend "[0] " to string, if missing */
                    const char sequence[] = "[0] ";
                    bool_t closed = (buf[s - 1] == '\n'); /* 命令是否结束？ */

                    if (buf[0] != '[' && (space - s) >= strlen(sequence) && isgraph(buf[0]) && buf[0] != '#' && closed
                        && epGtw->freshCommand) {
                        CO_GTWA_write(co->gtwa, sequence, strlen(sequence));
                    }
                    epGtw->freshCommand = closed;
                    CO_GTWA_write(co->gtwa, buf, s);
                } else { /* 套接字模式：本地或 TCP */
                    if (s == 0) {
                        /* 收到 EOF，关闭连接并启用套接字接受 */
                        /* EOF received, close connection and enable socket
                         * accepting */
                        int ret = epoll_ctl(ep->epoll_fd, EPOLL_CTL_DEL, epGtw->gtwa_fd, NULL);
                        if (ret < 0) {
                            log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(del, gtwa_fd)");
                        }
                        if (close(epGtw->gtwa_fd) < 0) {
                            log_printf(LOG_CRIT, DBG_ERRNO, "close(gtwa_fd)");
                        }
                        epGtw->gtwa_fd = -1;
                        socketAcceptEnableForEpoll(epGtw);
                    } else {
                        CO_GTWA_write(co->gtwa, buf, s);
                    }
                }
            }
            /* 重置超时计时器 */
            epGtw->socketTimeoutTmr_us = 0;

            ep->epoll_new = false;
        } else if ((ep->ev.events & (EPOLLERR | EPOLLHUP)) != 0) {
            /* 事件类型 C：套接字错误或挂断事件 */
            log_printf(LOG_DEBUG, DBG_GENERAL, "socket error or hangup, event=", ep->ev.events);
            if (close(epGtw->gtwa_fd) < 0) {
                log_printf(LOG_CRIT, DBG_ERRNO, "close(gtwa_fd, hangup)");
            }
        }
    } /* if (ep->epoll_new) */

    /* 如果建立了套接字连接，验证超时 */
    /* if socket connection is established, verify timeout */
    if (epGtw->socketTimeout_us > 0 && epGtw->gtwa_fdSocket > 0 && epGtw->gtwa_fd > 0) {
        if (epGtw->socketTimeoutTmr_us > epGtw->socketTimeout_us) {
            /* 超时过期，关闭当前连接并接受下一个连接 */
            /* timout expired, close current connection and accept next */
            int ret = epoll_ctl(ep->epoll_fd, EPOLL_CTL_DEL, epGtw->gtwa_fd, NULL);
            if (ret < 0) {
                log_printf(LOG_CRIT, DBG_ERRNO, "epoll_ctl(del, gtwa_fd), tmo");
            }
            if (close(epGtw->gtwa_fd) < 0) {
                log_printf(LOG_CRIT, DBG_ERRNO, "close(gtwa_fd), tmo");
            }
            epGtw->gtwa_fd = -1;
            socketAcceptEnableForEpoll(epGtw);
        } else {
            /* 累加经过的时间 */
            epGtw->socketTimeoutTmr_us += ep->timeDifference_us;
        }
    }
}
#endif /* (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII */
