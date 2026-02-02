/* CANopenNode Linux socketCAN 错误处理
 * 本文件提供 Linux socketCAN 平台的错误处理功能，包括 CAN 接口状态管理、
 * 错误消息处理、总线离线检测、无应答计数等。支持监听模式和错误恢复机制。
 */
/**
 * CANopenNode Linux socketCAN Error handling.
 *
 * @file        CO_error.h
 * @ingroup     CO_socketCAN_ERROR
 * @author      Martin Wagner
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

#ifndef CO_ERROR_H
#define CO_ERROR_H

#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <linux/can.h>
#include <net/if.h>

#if __has_include("CO_error_custom.h")
#include "CO_error_custom.h"
#else
#include "CO_error_msgs.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_socketCAN_ERROR CAN errors & Log
 * CANopen Errors and System message log
 *
 * @ingroup CO_socketCAN
 * @{
 */

/* 消息日志记录函数
 * 函数功能：记录系统日志消息，必须由应用程序定义实现。应该将日志消息记录到某个位置，
 *         例如 Linux 中的 syslog() 调用或 CANopen 网关的日志功能
 * 使用说明：默认情况下系统将消息存储在 /var/log/syslog 文件中。可以在使用前配置日志，
 *         例如过滤掉低于 LOG_NOTICE 的错误、指定程序名、打印进程 PID、同时打印到标准错误输出：
 *         setlogmask (LOG_UPTO (LOG_NOTICE));
 *         openlog ("exampleprog", LOG_PID | LOG_PERROR, LOG_USER);
 * 参数说明：
 *   - priority: 日志优先级，可选值为 LOG_EMERG、LOG_ALERT、LOG_CRIT、LOG_ERR、
 *              LOG_WARNING、LOG_NOTICE、LOG_INFO、LOG_DEBUG
 *   - format: 格式化字符串（类似 printf）
 *   - ...: 可变参数列表
 * 返回值说明：无返回值
 */
/**
 * Message logging function.
 *
 * Function must be defined by application. It should record log message to some place, for example syslog() call in
 * Linux or logging functionality in CANopen gateway @ref CO_CANopen_309_3.
 *
 * By default system stores messages in /var/log/syslog file. Log can optionally be configured before, for example to
 * filter out less critical errors than LOG_NOTICE, specify program name, print also process PID and print also to
 * standard error, set 'user' type of program, use:
 * @code
setlogmask (LOG_UPTO (LOG_NOTICE));
openlog ("exampleprog", LOG_PID | LOG_PERROR, LOG_USER);
 * @endcode
 *
 * @param priority one of LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR, LOG_WARNING, LOG_NOTICE, LOG_INFO, LOG_DEBUG
 * @param format format string as in printf
 */
void log_printf(int priority, const char* format, ...);

/* 驱动接口状态枚举
 * 枚举说明：CAN 硬件可以处于以下状态
 * 枚举值说明：
 *   - CO_INTERFACE_ACTIVE: CAN 错误被动/主动状态（正常工作）
 *   - CO_INTERFACE_LISTEN_ONLY: CAN 错误被动/主动，但当前总线上没有其他设备（监听模式）
 *   - CO_INTERFACE_BUS_OFF: CAN 总线离线（对总线没有影响）
 */
/**
 * driver interface state
 *
 * CAN hardware can be in the followning states:
 * - error active   (OK)
 * - error passive  (Can't generate error flags)
 * - bus off        (no influence on bus)
 */
typedef enum {
    CO_INTERFACE_ACTIVE,      /**< CAN error passive/active */
    CO_INTERFACE_LISTEN_ONLY, /**< CAN error passive/active, but currently no other device on bus */
    CO_INTERFACE_BUS_OFF      /**< CAN bus off */
} CO_CANinterfaceState_t;

/* 无应答最大次数常量
 * 说明：连续接收到多少次无应答（NO-ACK）后，假定总线上没有其他节点连接，
 *      因此进入监听模式
 */
/**
 * This is how many NO-ACKs need to be received in a row to assume that no other nodes are connected to a bus and
 * therefore are assuming listen-only
 */
#define CO_CANerror_NOACK_MAX   16

/* 监听模式阻塞时间常量
 * 说明：如果监听模式处于激活状态，阻塞传输的时长（以秒为单位）
 */
/**
 * This is how long we are going to block transmission if listen-only mode is active
 *
 * Time is in seconds.
 */
#define CO_CANerror_LISTEN_ONLY 10

/* socketCAN 接口错误处理结构体
 * 结构说明：管理单个 socketCAN 接口的错误检测和处理状态
 * 成员说明：
 *   - fd: 接口文件描述符
 *   - ifName: 接口名称字符串
 *   - noackCounter: CAN 传输无应答计数器
 *   - listenOnly: 设置为监听模式标志（易失性）
 *   - timestamp: 监听模式开始的时间戳
 *   - CANerrorStatus: CAN 错误状态位字段，参见 CO_CAN_ERR_status_t
 */
/**
 * socketCAN interface error handling
 */
typedef struct {
    int fd;                            /**< interface FD */
    char ifName[IFNAMSIZ];             /**< interface name as string */
    uint32_t noackCounter;             /**< counts no ACK on CAN transmission */
    volatile unsigned char listenOnly; /**< set to listen only mode */
    struct timespec timestamp;         /**< listen only mode started at this time */
    uint16_t CANerrorStatus;           /**< CAN error status bitfield, see @ref CO_CAN_ERR_status_t */
} CO_CANinterfaceErrorhandler_t;

/* 初始化 CAN 错误处理器
 * 函数功能：初始化 CAN 接口的错误处理器，每个接口需要一个错误处理器
 * 参数说明：
 *   - CANerrorhandler: 要初始化的错误处理器对象
 *   - fd: 接口文件描述符
 *   - ifname: 接口名称字符串
 * 返回值说明：无返回值
 */
/**
 * Initialize CAN error handler
 *
 * We need one error handler per interface
 *
 * @param CANerrorhandler This object will be initialized.
 * @param fd interface file descriptor
 * @param ifname interface name as string
 */
void CO_CANerror_init(CO_CANinterfaceErrorhandler_t* CANerrorhandler, int fd, const char* ifname);

/* 重置 CAN 错误处理器
 * 函数功能：重置 CAN 错误处理器的状态，清除错误标志
 * 参数说明：
 *   - CANerrorhandler: CAN 错误处理器对象
 * 返回值说明：无返回值
 */
/**
 * Reset CAN error handler
 *
 * @param CANerrorhandler CAN error object.
 */
void CO_CANerror_disable(CO_CANinterfaceErrorhandler_t* CANerrorhandler);

/* 消息接收事件处理
 * 函数功能：当接收到消息时，表明至少有一个其他 CAN 模块已连接。函数会清除
 *         监听模式和无应答计数器错误标志
 * 参数说明：
 *   - CANerrorhandler: CAN 错误处理器对象
 * 返回值说明：无返回值
 */
/**
 * Message received event
 *
 * When a message is received at least one other CAN module is connected. Function clears listenOnly and noackCounter
 * error flags.
 *
 * @param CANerrorhandler CAN error object.
 */
void CO_CANerror_rxMsg(CO_CANinterfaceErrorhandler_t* CANerrorhandler);

/* 检查接口是否准备好消息传输
 * 函数功能：检查 CAN 接口是否可以发送消息。如果未准备好，则不能发送消息
 * 参数说明：
 *   - CANerrorhandler: CAN 错误处理器对象
 * 返回值说明：
 *   - CO_INTERFACE_ACTIVE: 消息传输就绪，可以发送
 *   - 其他状态: 接口未就绪，不能发送消息
 */
/**
 * Check if interface is ready for message transmission
 *
 * Message mustn't be transmitted if not ready.
 *
 * @param CANerrorhandler CAN error object.
 * @return CO_INTERFACE_ACTIVE message transmission ready
 */
CO_CANinterfaceState_t CO_CANerror_txMsg(CO_CANinterfaceErrorhandler_t* CANerrorhandler);

/* 错误消息接收事件处理
 * 函数功能：处理所有接收到的 CAN 错误消息，更新接口错误状态
 * 参数说明：
 *   - CANerrorhandler: CAN 错误处理器对象
 *   - msg: 接收到的错误消息
 * 返回值说明：返回当前接口状态 CO_CANinterfaceState_t
 */
/**
 * Error message received event
 *
 * This handles all received error messages.
 *
 * @param CANerrorhandler CAN error object.
 * @param msg received error message
 * @return #CO_CANinterfaceState_t
 */
CO_CANinterfaceState_t CO_CANerror_rxMsgError(CO_CANinterfaceErrorhandler_t* CANerrorhandler,
                                              const struct can_frame* msg);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_ERROR_H */
