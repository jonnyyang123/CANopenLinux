/*
 * CAN module object for Linux socketCAN Error handling.
 *
 * @file        CO_error.c
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <syslog.h>
#include <time.h>
#include <linux/can/error.h>

#include "CO_error.h"
#include "301/CO_driver.h"

/* 函数功能: 重置CAN接口并设置为仅监听模式
 * 执行步骤:
 *   步骤1: 记录调试日志信息
 *   步骤2: 获取当前单调时钟时间戳并标记为仅监听模式
 *   步骤3: 如果需要重置接口，执行ip命令先关闭后启动CAN接口
 *   步骤4: 返回仅监听状态
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - resetIf: 是否需要重置接口的布尔标志
 * 返回值说明: 返回CO_INTERFACE_LISTEN_ONLY状态
 */
/*
 * Reset CAN interface and set to listen only mode
 */
static CO_CANinterfaceState_t
CO_CANerrorSetListenOnly(CO_CANinterfaceErrorhandler_t* CANerrorhandler, bool_t resetIf) {
    log_printf(LOG_DEBUG, DBG_CAN_SET_LISTEN_ONLY, CANerrorhandler->ifName);

    /* 获取当前时间戳并设置仅监听标志 */
    clock_gettime(CLOCK_MONOTONIC, &CANerrorhandler->timestamp);
    CANerrorhandler->listenOnly = true;

    /* 如果需要重置接口，执行系统命令重启CAN接口 */
    if (resetIf) {
        int ret;
        char command[100];
        snprintf(command, sizeof(command),
                 "ip link set %s down && "
                 "ip link set %s up "
                 "&",
                 CANerrorhandler->ifName, CANerrorhandler->ifName);
        ret = system(command);
        if (ret < 0) {
            log_printf(LOG_DEBUG, DBG_ERRNO, "system()");
        }
    }

    return CO_INTERFACE_LISTEN_ONLY;
}

/* 函数功能: 清除仅监听模式
 * 执行步骤:
 *   步骤1: 记录调试日志
 *   步骤2: 清除仅监听标志
 *   步骤3: 重置时间戳为0
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 * 返回值说明: 无返回值
 */
/*
 * Clear listen only
 */
static void
CO_CANerrorClearListenOnly(CO_CANinterfaceErrorhandler_t* CANerrorhandler) {
    log_printf(LOG_DEBUG, DBG_CAN_CLR_LISTEN_ONLY, CANerrorhandler->ifName);

    CANerrorhandler->listenOnly = false;
    CANerrorhandler->timestamp.tv_sec = 0;
    CANerrorhandler->timestamp.tv_nsec = 0;
}

/* 函数功能: 检查并处理"总线关闭"(bus off)状态
 * 执行步骤:
 *   步骤1: 初始化返回状态为活动状态
 *   步骤2: 检查CAN错误ID是否包含总线关闭标志
 *   步骤3: 如果检测到总线关闭，记录日志并设置为仅监听模式
 *   步骤4: 设置总线关闭错误状态标志
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - msg: CAN错误帧指针
 * 返回值说明: 返回CAN接口状态(活动或仅监听)
 */
/*
 * Check and handle "bus off" state
 */
static CO_CANinterfaceState_t
CO_CANerrorBusoff(CO_CANinterfaceErrorhandler_t* CANerrorhandler, const struct can_frame* msg) {
    CO_CANinterfaceState_t result = CO_INTERFACE_ACTIVE;

    /* 检查是否发生总线关闭错误 */
    if ((msg->can_id & CAN_ERR_BUSOFF) != 0) {
        log_printf(LOG_NOTICE, CAN_BUSOFF, CANerrorhandler->ifName);

        /* 处理总线关闭状态: CAN接口由于总线故障(如CAN线短路)进入关闭状态
         * 需要重启接口并标记为仅监听模式，这是清空内核和硬件发送队列的唯一方法 */
        /* The can interface changed it's state to "bus off" (e.g. because of
         * a short on the can wires). We re-start the interface and mark it
         * "listen only".
         * Restarting the interface is the only way to clear kernel and hardware
         * tx queues */
        result = CO_CANerrorSetListenOnly(CANerrorhandler, true);
        CANerrorhandler->CANerrorStatus |= CO_CAN_ERRTX_BUS_OFF;
    }
    return result;
}

/* 函数功能: 检查并处理CAN控制器问题
 * 执行步骤:
 *   步骤1: 初始化返回状态为活动状态
 *   步骤2: 检查是否有控制器错误标志
 *   步骤3: 清除总线关闭标志
 *   步骤4: 根据不同的错误类型设置相应的错误状态(接收/发送被动、溢出、警告等)
 *   步骤5: 记录相应的日志信息
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - msg: CAN错误帧指针
 * 返回值说明: 返回CAN接口状态
 */
/*
 * Check and handle controller problems
 */
static CO_CANinterfaceState_t
CO_CANerrorCrtl(CO_CANinterfaceErrorhandler_t* CANerrorhandler, const struct can_frame* msg) {
    CO_CANinterfaceState_t result = CO_INTERFACE_ACTIVE;

    /* 控制器错误处理说明:
     * - 错误计数器(rec/tec)由CAN硬件内部处理，此处无需操作
     * - 对于缓冲区溢出无法处理，确认型CANopen协议会检测错误，非确认型协议需容错
     * - 无法获知控制器何时离开警告级别，因此不清除也不设置警告标志 */
    /* Control
     * - error counters (rec/tec) are handled inside CAN hardware, nothing
     *   to do in here
     * - we can't really do anything about buffer overflows here. Confirmed
     *   CANopen protocols will detect the error, non-confirmed protocols
     *   need to be error tolerant.
     * - There is no information, when CAN controller leaves warning level,
     *   so we can't clear it. So we also don't set it. */
    if ((msg->can_id & CAN_ERR_CRTL) != 0) {
        /* 清除总线关闭标志 */
        /* clear bus off here */
        CANerrorhandler->CANerrorStatus &= 0xFFFF ^ CO_CAN_ERRTX_BUS_OFF;

        /* 处理接收被动错误 */
        if ((msg->data[1] & CAN_ERR_CRTL_RX_PASSIVE) != 0) {
            log_printf(LOG_NOTICE, CAN_RX_PASSIVE, CANerrorhandler->ifName);
            CANerrorhandler->CANerrorStatus |= CO_CAN_ERRRX_PASSIVE;
            /* CANerrorhandler->CANerrorStatus |= CO_CAN_ERRRX_WARNING; */
        /* 处理发送被动错误 */
        } else if ((msg->data[1] & CAN_ERR_CRTL_TX_PASSIVE) != 0) {
            log_printf(LOG_NOTICE, CAN_TX_PASSIVE, CANerrorhandler->ifName);
            CANerrorhandler->CANerrorStatus |= CO_CAN_ERRTX_PASSIVE;
            /* CANerrorhandler->CANerrorStatus |= CO_CAN_ERRTX_WARNING; */
        /* 处理接收缓冲区溢出 */
        } else if ((msg->data[1] & CAN_ERR_CRTL_RX_OVERFLOW) != 0) {
            log_printf(LOG_NOTICE, CAN_RX_BUF_OVERFLOW, CANerrorhandler->ifName);
            CANerrorhandler->CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
        /* 处理发送缓冲区溢出 */
        } else if ((msg->data[1] & CAN_ERR_CRTL_TX_OVERFLOW) != 0) {
            log_printf(LOG_NOTICE, CAN_TX_BUF_OVERFLOW, CANerrorhandler->ifName);
            CANerrorhandler->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        /* 处理接收警告级别 */
        } else if ((msg->data[1] & CAN_ERR_CRTL_RX_WARNING) != 0) {
            log_printf(LOG_INFO, CAN_RX_LEVEL_WARNING, CANerrorhandler->ifName);
            /* 清除被动标志，设置警告标志 */
            /* clear passive flag, set warning */
            CANerrorhandler->CANerrorStatus &= 0x7FFF ^ CO_CAN_ERRRX_PASSIVE;
            /* CANerrorhandler->CANerrorStatus |= CO_CAN_ERRRX_WARNING; */
        /* 处理发送警告级别 */
        } else if ((msg->data[1] & CAN_ERR_CRTL_TX_WARNING) != 0) {
            log_printf(LOG_INFO, CAN_TX_LEVEL_WARNING, CANerrorhandler->ifName);
            /* 清除被动标志，设置警告标志 */
            /* clear passive flag, set warning */
            CANerrorhandler->CANerrorStatus &= 0x7FFF ^ CO_CAN_ERRTX_PASSIVE;
            /* CANerrorhandler->CANerrorStatus |= CO_CAN_ERRTX_WARNING; */
        }
#ifdef CAN_ERR_CRTL_ACTIVE
        else if ((msg->data[1] & CAN_ERR_CRTL_ACTIVE) != 0) {
            log_printf(LOG_NOTICE, CAN_TX_LEVEL_ACTIVE, CANerrorhandler->ifName);
        }
#endif
    }
    return result;
}

/* 函数功能: 检查并处理无应答(no-acknowledge)错误
 * 执行步骤:
 *   步骤1: 如果已经在仅监听模式，直接返回仅监听状态
 *   步骤2: 检查是否收到ACK错误标志
 *   步骤3: 如果检测到ACK错误，递增无应答计数器
 *   步骤4: 当计数器超过阈值时，设置为仅监听模式
 *   步骤5: 如果没有ACK错误，重置计数器
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - msg: CAN错误帧指针
 * 返回值说明: 返回CAN接口状态(活动或仅监听)
 */
/*
 * Check and handle controller problems
 */
static CO_CANinterfaceState_t
CO_CANerrorNoack(CO_CANinterfaceErrorhandler_t* CANerrorhandler, const struct can_frame* msg) {
    CO_CANinterfaceState_t result = CO_INTERFACE_ACTIVE;

    /* 如果已经在仅监听模式，直接返回 */
    if (CANerrorhandler->listenOnly) {
        return CO_INTERFACE_LISTEN_ONLY;
    }

    /* 检测发送时未收到ACK应答 */
    /* received no ACK on transmission */
    if ((msg->can_id & CAN_ERR_ACK) != 0) {
        CANerrorhandler->noackCounter++;
        /* 当无应答计数器超过最大阈值时，进入仅监听模式 */
        if (CANerrorhandler->noackCounter > CO_CANerror_NOACK_MAX) {
            log_printf(LOG_INFO, CAN_NOACK, CANerrorhandler->ifName);

            /* 处理持续性无应答错误: 当总线上没有其他CAN节点活动时会持续收到NO-ACK错误
             * (CAN规范中的错误计数例外情况1)
             * 需要从CAN硬件缓冲区中删除导致无应答的消息，可通过重置接口或在内核驱动中删除 */
            /* We get the NO-ACK error continuously when no other CAN node
             * is active on the bus (Error Counting exception 1 in CAN spec).
             * todo - you need to pull the message causing no-ack from the CAN
             * hardware buffer. This can be done by either resetting interface
             * in here or deleting it within Linux Kernel can driver  (set "false"). */
            result = CO_CANerrorSetListenOnly(CANerrorhandler, true);
        }
    } else {
        /* 未检测到ACK错误，重置计数器 */
        CANerrorhandler->noackCounter = 0;
    }
    return result;
}

/* 函数功能: 初始化CAN错误处理器
 * 执行步骤:
 *   步骤1: 检查错误处理器指针是否有效
 *   步骤2: 设置文件描述符和接口名称
 *   步骤3: 初始化所有计数器和标志为0/false
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - fd: CAN接口的文件描述符
 *   - ifName: CAN接口名称字符串
 * 返回值说明: 无返回值
 */
void
CO_CANerror_init(CO_CANinterfaceErrorhandler_t* CANerrorhandler, int fd, const char* ifName) {
    if (CANerrorhandler == NULL) {
        return;
    }

    CANerrorhandler->fd = fd;
    memcpy(CANerrorhandler->ifName, ifName, sizeof(CANerrorhandler->ifName));
    CANerrorhandler->noackCounter = 0;
    CANerrorhandler->listenOnly = false;
    CANerrorhandler->timestamp.tv_sec = 0;
    CANerrorhandler->timestamp.tv_nsec = 0;
    CANerrorhandler->CANerrorStatus = 0;
}

/* 函数功能: 禁用CAN错误处理器
 * 执行步骤:
 *   步骤1: 检查错误处理器指针是否有效
 *   步骤2: 将整个结构体清零
 *   步骤3: 设置文件描述符为-1(无效值)
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 * 返回值说明: 无返回值
 */
void
CO_CANerror_disable(CO_CANinterfaceErrorhandler_t* CANerrorhandler) {
    if (CANerrorhandler == NULL) {
        return;
    }

    memset(CANerrorhandler, 0, sizeof(*CANerrorhandler));
    CANerrorhandler->fd = -1;
}

/* 函数功能: 处理接收消息事件
 * 执行步骤:
 *   步骤1: 检查错误处理器指针是否有效
 *   步骤2: 如果在仅监听模式，立即清除该模式(表明有节点活动)
 *   步骤3: 重置无应答计数器
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 * 返回值说明: 无返回值
 */
void
CO_CANerror_rxMsg(CO_CANinterfaceErrorhandler_t* CANerrorhandler) {
    if (CANerrorhandler == NULL) {
        return;
    }

    /* 检测到有节点活动，可以立即退出仅监听模式 */
    /* someone is active, we can leave listen only immediately */
    if (CANerrorhandler->listenOnly) {
        CO_CANerrorClearListenOnly(CANerrorhandler);
    }
    CANerrorhandler->noackCounter = 0;
}

/* 函数功能: 检查是否可以发送消息
 * 执行步骤:
 *   步骤1: 检查错误处理器指针，无效则返回总线关闭状态
 *   步骤2: 如果在仅监听模式，检查超时时间
 *   步骤3: 如果超过仅监听超时时间，尝试清除仅监听模式
 *   步骤4: 否则继续保持仅监听状态
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 * 返回值说明: 返回CAN接口状态(活动/仅监听/总线关闭)
 */
CO_CANinterfaceState_t
CO_CANerror_txMsg(CO_CANinterfaceErrorhandler_t* CANerrorhandler) {
    struct timespec now;

    if (CANerrorhandler == NULL) {
        return CO_INTERFACE_BUS_OFF;
    }

    /* 如果处于仅监听模式，检查是否超时可以尝试恢复 */
    if (CANerrorhandler->listenOnly) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (CANerrorhandler->timestamp.tv_sec + CO_CANerror_LISTEN_ONLY < now.tv_sec) {
            /* 超时后尝试恢复: 可能有节点在等待LSS，发送任何消息都会被应答 */
            /* let's try that again. Maybe someone is waiting for LSS now. It
             * doesn't matter which message is sent, as all messages are ACKed. */
            CO_CANerrorClearListenOnly(CANerrorhandler);
            return CO_INTERFACE_ACTIVE;
        }
        return CO_INTERFACE_LISTEN_ONLY;
    }
    return CO_INTERFACE_ACTIVE;
}

/* 函数功能: 处理接收到的CAN错误消息
 * 执行步骤:
 *   步骤1: 检查错误处理器指针，无效则返回总线关闭状态
 *   步骤2: 记录完整的错误消息到调试日志
 *   步骤3: 依次处理各类错误: 总线关闭、控制器错误、无应答错误
 *   步骤4: 每种错误处理后检查返回状态，非活动状态则直接返回
 * 参数说明:
 *   - CANerrorhandler: CAN接口错误处理器指针
 *   - msg: CAN错误帧指针
 * 返回值说明: 返回处理后的CAN接口状态
 */
CO_CANinterfaceState_t
CO_CANerror_rxMsgError(CO_CANinterfaceErrorhandler_t* CANerrorhandler, const struct can_frame* msg) {
    if (CANerrorhandler == NULL) {
        return CO_INTERFACE_BUS_OFF;
    }

    CO_CANinterfaceState_t result;

    /* 将所有错误消息完整记录到调试日志，即使后续会进一步分析 */
    /* Log all error messages in full to debug log, even if analysis is done
     * further on. */
    log_printf(LOG_DEBUG, DBG_CAN_ERROR_GENERAL, (int)msg->can_id, msg->data[0], msg->data[1], msg->data[2],
               msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], CANerrorhandler->ifName);

    /* 处理错误 - 从最明确的错误开始处理 */
    /* Process errors - start with the most unambiguous one */

    result = CO_CANerrorBusoff(CANerrorhandler, msg);
    if (result != CO_INTERFACE_ACTIVE) {
        return result;
    }

    result = CO_CANerrorCrtl(CANerrorhandler, msg);
    if (result != CO_INTERFACE_ACTIVE) {
        return result;
    }

    result = CO_CANerrorNoack(CANerrorhandler, msg);
    if (result != CO_INTERFACE_ACTIVE) {
        return result;
    }

    return CO_INTERFACE_ACTIVE;
}
