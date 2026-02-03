/* CANopenNode Linux socketCAN 错误处理的消息定义
 * 本文件定义了 Linux socketCAN 驱动的所有错误消息和通知消息的格式字符串。
 * 这些消息用于日志记录和调试，帮助开发者诊断 CAN 通信问题。
 */
/*
 * Definitions for CANopenNode Linux socketCAN Error handling.
 *
 * @file        CO_Error_msgs.h
 * @author      Martin Wagner
 * @copyright   2020 Neuberger Gebaeudeautomation GmbH
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

#ifndef CO_ERROR_MSGS_H
#define CO_ERROR_MSGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Linux CANopen socket 驱动的消息定义（通知和错误）
 * 说明：以下宏定义了各种 CAN 接口相关的错误和通知消息格式
 */
/*
 * Message definitions for Linux CANopen socket driver (notice and errors)
 */
/* CAN 接口未找到 */
#define CAN_NOT_FOUND                "(%s) CAN Interface \"%s\" not found", __func__
/* CAN 接口初始化失败 */
#define CAN_INIT_FAILED              "(%s) CAN Interface  \"%s\" Init failed", __func__
/* CAN 接口绑定失败 */
#define CAN_BINDING_FAILED           "(%s) Binding CAN Interface \"%s\" failed", __func__
/* 设置 CAN 接口错误过滤器失败 */
#define CAN_ERROR_FILTER_FAILED      "(%s) Setting CAN Interface \"%s\" error filter failed", __func__
/* 设置 CAN 接口消息过滤器失败 */
#define CAN_FILTER_FAILED            "(%s) Setting CAN Interface \"%s\" message filter failed", __func__
/* CAN 接口名称到索引的映射 */
#define CAN_NAMETOINDEX              "CAN Interface \"%s\" -> Index %d"
/* CAN 接口接收缓冲区大小设置 */
#define CAN_SOCKET_BUF_SIZE          "CAN Interface \"%s\" RX buffer set to %d messages (%d Bytes)"
/* CAN 接口接收队列溢出，丢失消息 */
#define CAN_RX_SOCKET_QUEUE_OVERFLOW "CAN Interface \"%s\" has lost %d messages"
/* CAN 接口进入总线离线状态，切换到监听模式 */
#define CAN_BUSOFF                   "CAN Interface \"%s\" changed to \"Bus Off\". Switching to Listen Only mode..."
/* CAN 接口未收到应答，切换到监听模式 */
#define CAN_NOACK                    "CAN Interface \"%s\" no \"ACK\" received.  Switching to Listen Only mode..."
/* CAN 接口状态变为接收被动 */
#define CAN_RX_PASSIVE               "CAN Interface \"%s\" changed state to \"Rx Passive\""
/* CAN 接口状态变为发送被动 */
#define CAN_TX_PASSIVE               "CAN Interface \"%s\" changed state to \"Tx Passive\""
/* CAN 接口状态变为主动 */
#define CAN_TX_LEVEL_ACTIVE          "CAN Interface \"%s\" changed state to \"Active\""
/* CAN 接口接收缓冲区溢出，消息被丢弃 */
#define CAN_RX_BUF_OVERFLOW          "CAN Interface \"%s\" Rx buffer overflow. Message dropped"
/* CAN 接口发送缓冲区溢出，消息被丢弃 */
#define CAN_TX_BUF_OVERFLOW          "CAN Interface \"%s\" Tx buffer overflow. Message dropped"
/* CAN 接口达到接收警告级别 */
#define CAN_RX_LEVEL_WARNING         "CAN Interface \"%s\" reached Rx Warning Level"
/* CAN 接口达到发送警告级别 */
#define CAN_TX_LEVEL_WARNING         "CAN Interface \"%s\" reached Tx Warning Level"

/* 调试用消息定义
 * 说明：以下宏定义了用于调试的各种消息格式
 */
/*
 * Message definitions for debugging
 */
/* 一般错误信息 */
#define DBG_GENERAL                  "(%s) Error: %s%d", __func__
/* 操作系统错误信息 */
#define DBG_ERRNO                    "(%s) OS error \"%s\" in %s", __func__, strerror(errno)
/* CANopen 调试信息 */
#define DBG_CO_DEBUG                 "(%s) CO_DEBUG: %s", __func__
/* CAN 消息发送失败 */
#define DBG_CAN_TX_FAILED            "(%s) Transmitting CAN msg OID 0x%03x failed(%s)", __func__
/* 设置 CAN 接收缓冲区失败 */
#define DBG_CAN_RX_PARAM_FAILED      "(%s) Setting CAN rx buffer failed (%s)", __func__
/* CAN 消息接收失败 */
#define DBG_CAN_RX_FAILED            "(%s) Receiving CAN msg failed (%s)", __func__
/* CAN 套接字错误消息（通用） */
#define DBG_CAN_ERROR_GENERAL                                                                                          \
    "(%s) Socket error msg ID: 0x%08x, Data[0..7]: 0x%02x, 0x%02x, 0x%02x, 0x%02x,"                                    \
    " 0x%02x, 0x%02x, 0x%02x, 0x%02x (%s)",                                                                            \
        __func__
/* CAN Epoll 错误 */
#define DBG_CAN_RX_EPOLL        "(%s) CAN Epoll error (0x%02x - %s)", __func__
/* 设置监听模式 */
#define DBG_CAN_SET_LISTEN_ONLY "(%s) %s Set Listen Only", __func__
/* 退出监听模式 */
#define DBG_CAN_CLR_LISTEN_ONLY "(%s) %s Leave Listen Only", __func__

/* 主线程相关的消息定义
 * 说明：以下宏定义了主程序中使用的各种消息格式
 */
/* mainline */
/* CANopen 紧急消息（来自远程节点） */
#define DBG_EMERGENCY_RX                                                                                               \
    "CANopen Emergency message from node 0x%02X: errorCode=0x%04X, errorRegister=0x%02X, errorBit=0x%02X, "            \
    "infoCode=0x%08X"
/* CANopen NMT 状态变化 */
#define DBG_NMT_CHANGE         "CANopen NMT state changed to: \"%s\" (%d)"
/* CANopen 远程节点的 NMT 状态变化（心跳消费者） */
#define DBG_HB_CONS_NMT_CHANGE "CANopen Remote node ID = 0x%02X (index = %d): NMT state changed to: \"%s\" (%d)"
/* 未知参数 */
#define DBG_ARGUMENT_UNKNOWN   "(%s) Unknown %s argument: \"%s\"", __func__
/* 不是有效的 TCP 端口 */
#define DBG_NOT_TCP_PORT       "(%s) -c argument \"%s\" is not a valid tcp port", __func__
/* 错误的节点 ID */
#define DBG_WRONG_NODE_ID      "(%s) Wrong node ID \"%d\"", __func__
/* 错误的实时优先级 */
#define DBG_WRONG_PRIORITY     "(%s) Wrong RT priority \"%d\"", __func__
/* 找不到 CAN 设备 */
#define DBG_NO_CAN_DEVICE      "(%s) Can't find CAN device \"%s\"", __func__
/* 存储错误 */
#define DBG_STORAGE            "(%s) Error with storage \"%s\"", __func__
/* 对象字典条目错误 */
#define DBG_OD_ENTRY           "(%s) Error in Object Dictionary entry: 0x%X", __func__
/* CANopen 错误 */
#define DBG_CAN_OPEN           "(%s) CANopen error in %s, err=%d", __func__
/* CANopen 设备信息 */
#define DBG_CAN_OPEN_INFO      "CANopen device, Node ID = 0x%02X, %s"

/* CO_epoll_interface 相关的消息定义
 * 说明：以下宏定义了 epoll 接口和命令接口相关的消息格式
 */
/* CO_epoll_interface */
/* Epoll 未知错误 */
#define DBG_EPOLL_UNKNOWN      "(%s) CAN Epoll error, events=0x%02x, fd=%d", __func__
/* 本地套接字绑定失败 */
#define DBG_COMMAND_LOCAL_BIND "(%s) Can't bind local socket to path \"%s\"", __func__
/* TCP 套接字绑定失败 */
#define DBG_COMMAND_TCP_BIND   "(%s) Can't bind tcp socket to port \"%d\"", __func__
/* 标准 I/O 命令接口启动信息 */
#define DBG_COMMAND_STDIO_INFO "CANopen command interface on \"standard IO\" started"
/* 本地套接字命令接口启动信息 */
#define DBG_COMMAND_LOCAL_INFO "CANopen command interface on local socket \"%s\" started"
/* TCP 套接字命令接口启动信息 */
#define DBG_COMMAND_TCP_INFO   "CANopen command interface on tcp port \"%d\" started"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_ERROR_MSGS_H */
