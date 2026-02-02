/* CANopenNode Linux socketCAN 平台驱动定义
 * 本文件包含 Linux socketCAN 平台特定的驱动定义和配置。包括 CAN 消息结构定义、
 * 多接口支持、错误报告、字节序处理等平台相关的核心功能定义。这是 CANopenNode
 * 协议栈与 Linux socketCAN 驱动的接口层。
 */
/**
 * Linux socketCAN specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @ingroup     CO_socketCAN_driver_target
 * @author      Janez Paternoster
 * @author      Martin Wagner
 * @copyright   2004 - 2020 Janez Paternoster
 * @copyright   2018 - 2020 Neuberger Gebaeudeautomation GmbH
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

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions. It is included from CO_driver.h, which contains
 * documentation for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <endian.h>
#ifndef CO_SINGLE_THREAD
#include <pthread.h>
#endif
#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif
#include "CO_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values. For more information see file CO_config.h. */
#ifdef CO_SINGLE_THREAD
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE 0
#else
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE CO_CONFIG_FLAG_CALLBACK_PRE
#endif
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT CO_CONFIG_FLAG_TIMERNEXT

#ifndef CO_CONFIG_NMT
#define CO_CONFIG_NMT                                                                                                  \
    (CO_CONFIG_NMT_CALLBACK_CHANGE | CO_CONFIG_NMT_MASTER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                         \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS                                                                                              \
    (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_CALLBACK_CHANGE | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                 \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_EM
#define CO_CONFIG_EM                                                                                                   \
    (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_PROD_CONFIGURABLE | CO_CONFIG_EM_PROD_INHIBIT | CO_CONFIG_EM_HISTORY         \
     | CO_CONFIG_EM_STATUS_BITS | CO_CONFIG_EM_CONSUMER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                           \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_SDO_SRV
#define CO_CONFIG_SDO_SRV                                                                                              \
    (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_SDO_SRV_BLOCK | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                        \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_SDO_SRV_BUFFER_SIZE
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 900
#endif

#ifndef CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI                                                                                              \
    (CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_SDO_CLI_BLOCK | CO_CONFIG_SDO_CLI_LOCAL        \
     | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_TIME
#define CO_CONFIG_TIME                                                                                                 \
    (CO_CONFIG_TIME_ENABLE | CO_CONFIG_TIME_PRODUCER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                              \
     | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_CONFIG_LSS
#define CO_CONFIG_LSS                                                                                                  \
    (CO_CONFIG_LSS_SLAVE | CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND | CO_CONFIG_LSS_MASTER                          \
     | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)
#endif

#ifndef CO_CONFIG_GFC
#define CO_CONFIG_GFC (CO_CONFIG_GFC_ENABLE | CO_CONFIG_GFC_CONSUMER | CO_CONFIG_GFC_PRODUCER)
#endif

#ifndef CO_CONFIG_SRDO
#define CO_CONFIG_SRDO                                                                                                 \
    (CO_CONFIG_SRDO_ENABLE | CO_CONFIG_SRDO_CHECK_TX | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                              \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_GTW
#define CO_CONFIG_GTW                                                                                                  \
    (CO_CONFIG_GTW_ASCII | CO_CONFIG_GTW_ASCII_SDO | CO_CONFIG_GTW_ASCII_NMT | CO_CONFIG_GTW_ASCII_LSS                 \
     | CO_CONFIG_GTW_ASCII_LOG | CO_CONFIG_GTW_ASCII_ERROR_DESC | CO_CONFIG_GTW_ASCII_PRINT_HELP                       \
     | CO_CONFIG_GTW_ASCII_PRINT_LEDS)
#define CO_CONFIG_GTW_BLOCK_DL_LOOP  3
#define CO_CONFIG_GTWA_COMM_BUF_SIZE 2000
#define CO_CONFIG_GTWA_LOG_BUF_SIZE  10000
#endif

#ifndef CO_CONFIG_CRC16
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE)
#endif

#ifndef CO_CONFIG_FIFO
#define CO_CONFIG_FIFO                                                                                                 \
    (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_ALT_READ | CO_CONFIG_FIFO_CRC16_CCITT | CO_CONFIG_FIFO_ASCII_COMMANDS      \
     | CO_CONFIG_FIFO_ASCII_DATATYPES)
#endif

/* Print debug info from some internal parts of the stack */
#if (CO_CONFIG_DEBUG) & CO_CONFIG_DEBUG_COMMON
#include <stdio.h>
#include <syslog.h>
#define CO_DEBUG_COMMON(msg) log_printf(LOG_DEBUG, DBG_CO_DEBUG, msg);
#endif

/**
 * @defgroup CO_socketCAN_driver_target CO_driver_target.h
 * Linux socketCAN specific @ref CO_driver definitions for CANopenNode.
 *
 * @ingroup CO_socketCAN
 * @{
 */

/* 多接口支持配置宏
 * 功能说明：启用此宏可在驱动层使用接口组合功能，允许在指定的接口上广播/选择性发送消息，
 *         并将所有接收到的消息合并到一个队列中
 * 配置说明：
 *   - 设置为 0：CO_CANmodule_init() 使用 CANptr 参数添加单个 socketCAN 接口，失败时返回 CO_ERROR_SYSCALL
 *   - 设置为 1：CO_CANmodule_init() 忽略 CANptr 参数，必须在初始化后通过 CO_CANmodule_addInterface() 函数添加接口
 * 默认值：0（禁用），可以被覆盖
 * 注意：这不是用于实现接口冗余的功能
 */
/**
 * Multi interface support
 *
 * Enable this to use interface combining at driver level. This adds functions to broadcast/selective transmit messages
 * on the given interfaces as well as combining all received message into one queue.
 *
 * If CO_DRIVER_MULTI_INTERFACE is set to 0, then CO_CANmodule_init() adds single socketCAN interface specified by
 * CANptr argument. In case of failure, CO_CANmodule_init() returns CO_ERROR_SYSCALL.
 *
 * If CO_DRIVER_MULTI_INTERFACE is set to 1, then CO_CANmodule_init() ignores CANptr argument. Interfaces must be added
 * by CO_CANmodule_addInterface() function after CO_CANmodule_init().
 *
 * Macro is set to 0 (disabled) by default. It can be overridden.
 *
 * This is not intended to realize interface redundancy!!!
 */
#ifndef CO_DRIVER_MULTI_INTERFACE
#define CO_DRIVER_MULTI_INTERFACE 0
#endif

/* CAN 总线错误报告配置宏
 * 功能说明：启用此宏可在驱动中添加 socketCAN 错误检测和处理功能，这在需要支持
 *         "0 个连接节点"的使用场景时是必需的（虽然这在 CAN 中通常是被禁止的）
 * 配置说明：需要在内核驱动中启用错误报告，使用命令：
 *         ip link set canX type can berr-reporting on
 * 默认值：1（启用），可以被覆盖
 * 注意：硬件的内核驱动需要实现此功能才能正常工作
 */
/**
 * CAN bus error reporting
 *
 * CO_DRIVER_ERROR_REPORTING enabled adds support for socketCAN error detection and handling functions inside the
 * driver. This is needed when you have CANopen with "0" connected nodes as a use case, as this is normally forbidden
 * in CAN.
 *
 * Macro is set to 1 (enabled) by default. It can be overridden.
 *
 * you need to enable error reporting in your kernel driver using:
 * @code{.sh}
ip link set canX type can berr-reporting on
 * @endcode
 * Of course, the kernel driver for your hardware needs this functionality to be
 * implemented...
 */
#ifndef CO_DRIVER_ERROR_REPORTING
#define CO_DRIVER_ERROR_REPORTING 1
#endif

/* skip this section for Doxygen, because it is documented in CO_driver.h */
#ifndef CO_DOXYGEN

/* Basic definitions */
#ifdef __BYTE_ORDER
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
#else
#define CO_BIG_ENDIAN
#include <byteswap.h>
#define CO_SWAP_16(x) bswap_16(x)
#define CO_SWAP_32(x) bswap_32(x)
#define CO_SWAP_64(x) bswap_64(x)
#endif
#endif
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* CAN 接收消息结构体（按照 socketCAN 对齐）
 * 结构说明：定义 CAN 接收消息的数据结构，与 Linux socketCAN 的内存布局对齐
 * 成员说明：
 *   - ident: CAN 标识符（11 位标准帧或 29 位扩展帧）
 *   - DLC: 数据长度代码，表示数据字节数（0-8）
 *   - padding: 填充字节，用于内存对齐
 *   - data: 实际的 CAN 数据字节（最多 8 字节）
 */
/* CAN receive message structure as aligned in socketCAN. */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t padding[3];
    uint8_t data[8];
} CO_CANrxMsg_t;

/* Access to received CAN message */
static inline uint16_t
CO_CANrxMsg_readIdent(void* rxMsg) {
    CO_CANrxMsg_t* rxMsgCasted = (CO_CANrxMsg_t*)rxMsg;
    return (uint16_t)(rxMsgCasted->ident & CAN_SFF_MASK);
}

static inline uint8_t
CO_CANrxMsg_readDLC(void* rxMsg) {
    CO_CANrxMsg_t* rxMsgCasted = (CO_CANrxMsg_t*)rxMsg;
    return (uint8_t)(rxMsgCasted->DLC);
}

static inline uint8_t*
CO_CANrxMsg_readData(void* rxMsg) {
    CO_CANrxMsg_t* rxMsgCasted = (CO_CANrxMsg_t*)rxMsg;
    return (uint8_t*)(rxMsgCasted->data);
}

/* 接收消息对象结构体
 * 结构说明：定义 CAN 接收消息的完整对象，包含标识符、掩码、回调函数和时间戳等信息
 * 成员说明：
 *   - ident: CAN 标识符，用于消息过滤
 *   - mask: 标识符掩码，用于消息过滤
 *   - object: 与此接收缓冲区关联的对象指针
 *   - CANrx_callback: 接收到匹配消息时调用的回调函数
 *   - can_ifindex: 最后一条消息接收时的 CAN 接口索引
 *   - timestamp: 最后一条消息的接收时间戳（系统时钟）
 */
/* Received message object */
typedef struct {
    uint32_t ident;
    uint32_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
    int can_ifindex;           /* CAN Interface index from last message */
    struct timespec timestamp; /* time of reception of last message */
} CO_CANrx_t;

/* 发送消息对象结构体（按照 socketCAN 对齐）
 * 结构说明：定义 CAN 发送消息的数据结构，与 Linux socketCAN 的内存布局对齐
 * 成员说明：
 *   - ident: CAN 标识符
 *   - DLC: 数据长度代码，表示数据字节数（0-8）
 *   - padding: 填充字节，确保内存对齐
 *   - data: 实际的 CAN 数据字节（最多 8 字节）
 *   - bufferFull: 缓冲区已满标志（易失性，可被中断修改）
 *   - syncFlag: 关于发送消息的同步信息标志（易失性）
 *   - can_ifindex: 要使用的 CAN 接口索引
 */
/* Transmit message object as aligned in socketCAN. */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t padding[3]; /* ensure alignment */
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag; /* info about transmit message */
    int can_ifindex;          /* CAN Interface index to use */
} CO_CANtx_t;

/* 标准帧格式的最大 COB ID 常量
 * 说明：标准 CAN 帧（SFF）的最大通信对象标识符（COB ID）
 */
/* Max COB ID for standard frame format */
#define CO_CAN_MSG_SFF_MAX_COB_ID (1 << CAN_SFF_ID_BITS)

/* CAN 接口对象（CANptr），传递给 CO_CANinit() 函数
 * 结构说明：定义传递给 CAN 初始化函数的接口参数
 * 成员说明：
 *   - can_ifindex: CAN 接口索引
 *   - epoll_fd: epoll 文件描述符，用于等待 CAN 接收事件
 */
/* CAN interface object (CANptr), passed to CO_CANinit() */
typedef struct {
    int can_ifindex; /* CAN Interface index */
    int epoll_fd;    /* File descriptor for epoll, which waits for CAN receive event */
} CO_CANptrSocketCan_t;

/* socketCAN 接口对象
 * 结构说明：表示一个 socketCAN 网络接口的完整信息
 * 成员说明：
 *   - can_ifindex: CAN 接口索引
 *   - ifName: CAN 接口名称（字符串形式，如 "can0"）
 *   - fd: socketCAN 文件描述符
 *   - errorhandler: CAN 接口错误处理器（仅在启用错误报告时可用）
 */
/* socketCAN interface object */
typedef struct {
    int can_ifindex;       /* CAN Interface index */
    char ifName[IFNAMSIZ]; /* CAN Interface name */
    int fd;                /* socketCAN file descriptor */
#if CO_DRIVER_ERROR_REPORTING > 0 || defined CO_DOXYGEN
    CO_CANinterfaceErrorhandler_t errorhandler;
#endif
} CO_CANinterface_t;

/* CAN 模块对象
 * 结构说明：CAN 驱动的主要对象，管理 CAN 接口、消息缓冲区和状态
 * 成员说明：
 *   - CANinterfaces: CAN 接口列表，来自 CO_CANmodule_init() 或每次 CO_CANmodule_addInterface() 调用
 *   - CANinterfaceCount: 接口数量
 *   - rxArray: 接收缓冲区数组
 *   - rxSize: 接收缓冲区大小
 *   - rxFilter: socketCAN 过滤器列表，每个接收缓冲区一个
 *   - rxDropCount: 在接收套接字队列中丢弃的消息数量
 *   - txArray: 发送缓冲区数组
 *   - txSize: 发送缓冲区大小
 *   - CANerrorStatus: CAN 错误状态
 *   - CANnormal: CAN 正常运行标志（易失性）
 *   - CANtxCount: CAN 发送计数（易失性）
 *   - epoll_fd: epoll 文件描述符，用于等待 CAN 接收事件
 *   - rxIdentToIndex: COB ID 到接收数组索引的查找表（仅用于标准帧消息，多接口模式）
 *   - txIdentToIndex: COB ID 到发送数组索引的查找表（仅用于标准帧消息，多接口模式）
 */
/* CAN module object */
typedef struct {
    /* List of can interfaces. From CO_CANmodule_init() or one per CO_CANmodule_addInterface() call */
    CO_CANinterface_t* CANinterfaces;
    uint32_t CANinterfaceCount; /* interface count */
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    struct can_filter* rxFilter; /* socketCAN filter list, one per rx buffer */
    uint32_t rxDropCount;        /* messages dropped on rx socket queue */
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile uint16_t CANtxCount;
    int epoll_fd; /* File descriptor for epoll, which waits for CAN receive event */
#if CO_DRIVER_MULTI_INTERFACE > 0 || defined CO_DOXYGEN
    /* Lookup tables Cob ID to rx/tx array index.  Only feasible for SFF Messages. */
    uint32_t rxIdentToIndex[CO_CAN_MSG_SFF_MAX_COB_ID];
    uint32_t txIdentToIndex[CO_CAN_MSG_SFF_MAX_COB_ID];
#endif
} CO_CANmodule_t;

/* 数据存储：最大文件名长度（包括路径）
 * 说明：定义存储数据时文件路径的最大长度
 */
/* Data storage: Maximum file name length including path */
#ifndef CO_STORAGE_PATH_MAX
#define CO_STORAGE_PATH_MAX 255
#endif

/* 单个数据存储条目对象
 * 结构说明：表示一个数据存储条目，包含数据地址、长度和文件信息
 * 成员说明：
 *   - addr: 数据在内存中的地址
 *   - len: 数据长度（字节数）
 *   - subIndexOD: 对象字典子索引
 *   - attr: 属性标志
 *   - filename: 存储数据块的文件名（包括路径）
 *   - crc: 之前存储的数据的 CRC 校验和，用于自动存储
 *   - fp: 已打开文件的指针，用于自动存储
 */
/* Data storage object for one entry */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    char filename[CO_STORAGE_PATH_MAX]; /* Name of the file, where data block is stored */
    uint16_t crc;                       /* CRC checksum of the data stored previously, for auto storage */
    FILE* fp;                           /* Pointer to opened file, for auto storage */
} CO_storage_entry_t;

#ifdef CO_SINGLE_THREAD
#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                                                   \
    { (void)CAN_MODULE; }
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)                                                                                 \
    { (void)CAN_MODULE; }
#define CO_LOCK_EMCY(CAN_MODULE)                                                                                       \
    { (void)CAN_MODULE; }
#define CO_UNLOCK_EMCY(CAN_MODULE)                                                                                     \
    { (void)CAN_MODULE; }
#define CO_LOCK_OD(CAN_MODULE)                                                                                         \
    { (void)CAN_MODULE; }
#define CO_UNLOCK_OD(CAN_MODULE)                                                                                       \
    { (void)CAN_MODULE; }
#define CO_MemoryBarrier()
#else

/* (un)lock critical section in CO_CANsend() - unused */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
extern pthread_mutex_t CO_EMCY_mutex;

static inline int
CO_LOCK_EMCY(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    return pthread_mutex_lock(&CO_EMCY_mutex);
}

static inline void
CO_UNLOCK_EMCY(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    (void)pthread_mutex_unlock(&CO_EMCY_mutex);
}

/* (un)lock critical section when accessing Object Dictionary */
extern pthread_mutex_t CO_OD_mutex;

static inline int
CO_LOCK_OD(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    return pthread_mutex_lock(&CO_OD_mutex);
}

static inline void
CO_UNLOCK_OD(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    (void)pthread_mutex_unlock(&CO_OD_mutex);
}

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()                                                                                             \
    { __sync_synchronize(); }
#endif /* CO_SINGLE_THREAD */

#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    }
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    }

#endif /* #ifndef CO_DOXYGEN */

#if CO_DRIVER_MULTI_INTERFACE > 0 || defined CO_DOXYGEN
/* 向 CAN 驱动添加 socketCAN 接口
 * 函数功能：向已初始化的 CAN 模块添加额外的 socketCAN 接口（仅在多接口模式下可用）
 * 使用说明：必须在 CO_CANmodule_init() 之后调用
 * 参数说明：
 *   - CANmodule: 要添加接口的 CAN 模块对象
 *   - can_ifindex: 要添加的 CAN 接口索引
 * 返回值说明：
 *   - CO_ERROR_NO: 成功
 *   - CO_ERROR_ILLEGAL_ARGUMENT: 参数非法
 *   - CO_ERROR_SYSCALL: 系统调用失败
 *   - CO_ERROR_INVALID_STATE: 状态无效
 */
/**
 * Add socketCAN interface to can driver
 *
 * Function must be called after CO_CANmodule_init.
 *
 * @param CANmodule This object will be initialized.
 * @param can_ifindex CAN Interface index
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT, CO_ERROR_SYSCALL or CO_ERROR_INVALID_STATE.
 */
CO_ReturnError_t CO_CANmodule_addInterface(CO_CANmodule_t* CANmodule, int can_ifindex);

/* 查询消息缓冲区最后接收消息的接口
 * 函数功能：检查指定消息缓冲区最后一次接收消息时使用的接口
 * 使用说明：用户需要确保此信息对特定消息有意义，因为某些消息可能在任何时间从任何总线接收
 * 参数说明：
 *   - CANmodule: CAN 模块对象
 *   - ident: 11 位标准 CAN 标识符
 *   - CANptrRx: [输出] 消息接收的接口指针
 *   - timestamp: [输出] 消息接收的时间戳（系统时钟）
 * 返回值说明：
 *   - false: 消息从未被接收过，因此没有可用的基地址和时间戳
 *   - true: 基地址和时间戳有效
 */
/**
 * Check on which interface the last message for one message buffer was received
 *
 * It is in the responsibility of the user to check that this information is useful as some messages can be received at
 * any time on any bus.
 *
 * @param CANmodule This object.
 * @param ident 11-bit standard CAN Identifier.
 * @param [out] CANptrRx message was received on this interface
 * @param [out] timestamp message was received at this time (system clock)
 *
 * @retval false message has never been received, therefore no base address and timestamp are available
 * @retval true base address and timestamp are valid
 */
bool_t CO_CANrxBuffer_getInterface(CO_CANmodule_t* CANmodule, uint16_t ident, const void** const CANptrRx,
                                   struct timespec* timestamp);

/* 设置消息缓冲区的发送接口
 * 函数功能：设置指定消息缓冲区应使用的发送接口
 * 使用说明：用户需要确保使用正确的接口，某些消息需要在所有接口上发送
 * 参数说明：
 *   - CANmodule: CAN 模块对象
 *   - ident: 11 位标准 CAN 标识符
 *   - CANptrTx: 要使用的接口指针，NULL 表示未指定（将在所有可用接口上发送）
 * 返回值说明：
 *   - CO_ERROR_NO: 成功
 *   - CO_ERROR_ILLEGAL_ARGUMENT: 参数非法
 */
/**
 * Set which interface should be used for message buffer transmission
 *
 * It is in the responsibility of the user to ensure that the correct interface is used. Some messages need to be
 * transmitted on all interfaces.
 *
 * If given interface is unknown or NULL is used, a message is transmitted on all available interfaces.
 *
 * @param CANmodule This object.
 * @param ident 11-bit standard CAN Identifier.
 * @param CANptrTx use this interface. NULL = not specified
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_CANtxBuffer_setInterface(CO_CANmodule_t* CANmodule, uint16_t ident, const void* CANptrTx);
#endif /* CO_DRIVER_MULTI_INTERFACE */

/* 从 epoll 事件接收 CAN 消息
 * 函数功能：验证 epoll 事件是否匹配任何 CAN 接口事件，如果匹配则读取并预处理 CAN 消息
 *         也会处理 CAN 错误帧
 * 使用说明：此函数可以两种方式结合使用：
 *         1. 自动模式：如果为匹配的 _rxArray_ 指定了 CANrx_callback，则自动调用其回调函数
 *         2. 手动模式：评估消息过滤器，返回接收到的消息
 * 参数说明：
 *   - CANmodule: CAN 模块对象
 *   - ev: 要验证匹配的 epoll 事件
 *   - buffer: [输出] 接收消息的存储位置，不使用时可以为 NULL
 *   - msgIndex: [输出] 接收消息在 CO_CANmodule_t 的 _rxArray_ 中的索引，
 *              接收到的 CAN 消息副本存储在 _buffer_ 中
 * 返回值说明：
 *   - true: epoll 事件匹配任何 CAN 接口
 *   - false: 事件不匹配
 */
/**
 * Receives CAN messages from matching epoll event
 *
 * This function verifies, if epoll event matches event from any CANinterface. In case of match, message is read from
 * CAN and pre-processed for CANopenNode objects. CAN error frames are also processed.
 *
 * In case of CAN message function searches _rxArray_ from CO_CANmodule_t and if matched it calls the corresponding
 * CANrx_callback, optionally copies received CAN message to _buffer_ and returns index of matched _rxArray_.
 *
 * This function can be used in two ways, which can be combined:
 * - automatic mode: If CANrx_callback is specified for matched _rxArray_, then   calls its callback.
 * - manual mode: evaluate message filters, return received message
 *
 * @param CANmodule This object.
 * @param ev Epoll event, which vill be verified for matches.
 * @param [out] buffer Storage for received message or _NULL_ if not used.
 * @param [out] msgIndex Index of received message in array from CO_CANmodule_t _rxArray_, copy of CAN message is
 * available in _buffer_.
 *
 * @return True, if epoll event matches any CAN interface.
 */
bool_t CO_CANrxFromEpoll(CO_CANmodule_t* CANmodule, struct epoll_event* ev, CO_CANrxMsg_t* buffer, int32_t* msgIndex);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
