/* Linux socketCAN 驱动程序 - CANopenNode 的底层 CAN 通信接口实现 */
/*
 * Linux socketCAN interface for CANopenNode.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster, Martin Wagner
 * @copyright   2004 - 2015 Janez Paternoster, 2017 - 2020 Neuberger Gebaeudeautomation GmbH
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
#include <unistd.h>
#include <errno.h>
#include <syslog.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/net_tstamp.h>
#include <sys/socket.h>
#include <asm/socket.h>
#include <sys/eventfd.h>
#include <time.h>

#include "301/CO_driver.h"
#include "CO_error.h"

#ifndef CO_SINGLE_THREAD
pthread_mutex_t CO_EMCY_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t CO_OD_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

#if CO_DRIVER_MULTI_INTERFACE == 0
static CO_ReturnError_t CO_CANmodule_addInterface(CO_CANmodule_t* CANmodule, int can_ifindex);
#endif

#if CO_DRIVER_MULTI_INTERFACE > 0

/* 无效的 COB-ID 标记值 */
static const uint32_t CO_INVALID_COB_ID = 0xffffffff;

/* 函数功能：设置 COB-ID 到缓冲区索引的映射关系
 * 执行步骤：
 *   步骤1: 如果 COB-ID 发生变化，先移除旧的映射
 *   步骤2: 检查新的 COB-ID 是否有效（不超过标准帧最大值）
 *   步骤3: 处理特殊情况 - COB-ID 0 仅在索引 0 时有效（用于 NMT）
 *   步骤4: 建立新的 COB-ID 到索引的映射
 * 参数说明：
 *   lookup - COB-ID 查找表指针，用于快速查找
 *   index - 缓冲区索引值
 *   identNew - 新的 COB-ID 标识符
 *   identCurrent - 当前的 COB-ID 标识符
 * 返回值说明：无返回值
 */
void
CO_CANsetIdentToIndex(uint32_t* lookup, uint32_t index, uint32_t identNew, uint32_t identCurrent) {
    /* 步骤1: 条目已更改，移除旧的映射 */
    /* entry changed, remove old one */
    if (identCurrent < CO_CAN_MSG_SFF_MAX_COB_ID && identNew != identCurrent) {
        lookup[identCurrent] = CO_INVALID_COB_ID;
    }

    /* 步骤2: 检查此 COB-ID 是否在标准帧范围内 */
    /* check if this COB ID is part of the table */
    if (identNew > CO_CAN_MSG_SFF_MAX_COB_ID) {
        return;
    }

    /* 步骤3: 特殊情况处理 - COB-ID "0" 在 *xArray[0] 中是有效值（用于 NMT），
     * 对于其他情况表示"未配置条目" */
    /* Special case COB ID "0" -> valid value in *xArray[0] (CO_*CAN_NMT),
     * "entry unconfigured" for all others */
    if (identNew == 0) {
        if (index == 0) {
            lookup[0] = 0;
        }
    } else {
        /* 步骤4: 建立新的映射关系 */
        lookup[identNew] = index;
    }
}

/* 函数功能：根据 COB-ID 标识符从查找表中获取对应的缓冲区索引
 * 参数说明：
 *   lookup - COB-ID 查找表指针
 *   ident - COB-ID 标识符
 * 返回值说明：返回对应的缓冲区索引，如果无效则返回 CO_INVALID_COB_ID
 */
static uint32_t
CO_CANgetIndexFromIdent(uint32_t* lookup, uint32_t ident) {
    /* 检查此 COB-ID 是否在有效范围内 */
    /* check if this COB ID is part of the table */
    if (ident > CO_CAN_MSG_SFF_MAX_COB_ID) {
        return CO_INVALID_COB_ID;
    }

    return lookup[ident];
}

#endif /* CO_DRIVER_MULTI_INTERFACE */

/* 函数功能：禁用 socketCAN 接收功能，停止接收所有 CAN 消息
 * 执行步骤：
 *   步骤1: 初始化返回值为无错误
 *   步骤2: 遍历所有 CAN 接口
 *   步骤3: 为每个接口设置空过滤器（不匹配任何消息），实现禁用接收
 *   步骤4: 记录设置失败的错误信息
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 * 返回值说明：
 *   CO_ERROR_NO - 成功禁用接收
 *   CO_ERROR_SYSCALL - 系统调用失败
 */
/* Disable socketCAN rx */
static CO_ReturnError_t
disableRx(CO_CANmodule_t* CANmodule) {
    uint32_t i;
    CO_ReturnError_t retval;

    /* 步骤3: 插入一个不匹配任何消息的过滤器（设置为 NULL 和长度 0）*/
    /* insert a filter that doesn't match any messages */
    retval = CO_ERROR_NO;
    for (i = 0; i < CANmodule->CANinterfaceCount; i++) {
        int ret = setsockopt(CANmodule->CANinterfaces[i].fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
        if (ret < 0) {
            /* 步骤4: 记录过滤器设置失败的错误 */
            log_printf(LOG_ERR, CAN_FILTER_FAILED, CANmodule->CANinterfaces[i].ifName);
            log_printf(LOG_DEBUG, DBG_ERRNO, "setsockopt()");
            retval = CO_ERROR_SYSCALL;
        }
    }

    return retval;
}

/* 函数功能：设置或更新 socketCAN 接收过滤器，控制接收哪些 CAN 消息
 * 执行步骤：
 *   步骤1: 创建过滤器副本数组
 *   步骤2: 复制有效的过滤器条目（排除 id 和 mask 都为 0 的条目）
 *   步骤3: 如果没有有效过滤器，则禁用接收
 *   步骤4: 为每个 CAN 接口应用过滤器设置
 *   步骤5: 处理设置失败的错误
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 * 返回值说明：
 *   CO_ERROR_NO - 成功设置过滤器
 *   CO_ERROR_SYSCALL - 系统调用失败
 */
/* Set up or update socketCAN rx filters */
static CO_ReturnError_t
setRxFilters(CO_CANmodule_t* CANmodule) {
    size_t i;
    int count;
    CO_ReturnError_t retval;

    struct can_filter rxFiltersCpy[CANmodule->rxSize];

    count = 0;
    /* 步骤2: 移除未使用的条目（id == 0 且 mask == 0），因为它们会作为"通过所有"过滤器 */
    /* remove unused entries ( id == 0 and mask == 0 ) as they would act as "pass all" filter */
    for (i = 0; i < CANmodule->rxSize; i++) {
        if ((CANmodule->rxFilter[i].can_id != 0) || (CANmodule->rxFilter[i].can_mask != 0)) {

            rxFiltersCpy[count] = CANmodule->rxFilter[i];

            count++;
        }
    }

    if (count == 0) {
        /* 步骤3: 没有设置过滤器，禁用接收 */
        /* No filter is set, disable RX */
        return disableRx(CANmodule);
    }

    /* 步骤4: 为所有接口应用过滤器 */
    retval = CO_ERROR_NO;
    for (i = 0; i < CANmodule->CANinterfaceCount; i++) {
        int ret = setsockopt(CANmodule->CANinterfaces[i].fd, SOL_CAN_RAW, CAN_RAW_FILTER, rxFiltersCpy,
                             sizeof(struct can_filter) * count);
        if (ret < 0) {
            /* 步骤5: 记录过滤器设置失败 */
            log_printf(LOG_ERR, CAN_FILTER_FAILED, CANmodule->CANinterfaces[i].ifName);
            log_printf(LOG_DEBUG, DBG_ERRNO, "setsockopt()");
            retval = CO_ERROR_SYSCALL;
        }
    }

    return retval;
}

/* 函数功能：设置 CAN 模块为配置模式（socketCAN 中此函数为空实现）
 * 参数说明：
 *   CANptr - CAN 指针（未使用）
 * 返回值说明：无返回值
 * 注意：由于未提供 CANmodule_t 引用，此函数无法执行任何操作
 */
void
CO_CANsetConfigurationMode(void* CANptr) {
    (void)CANptr;
    /* socketCAN 驱动中无法执行配置模式操作，因为未提供 CANmodule_t 引用 */
    /* Can't do anything because no reference to CANmodule_t is provided */
}

/* 函数功能：将 CAN 模块切换到正常工作模式，启用消息收发
 * 执行步骤：
 *   步骤1: 暂时设置模块为非正常模式
 *   步骤2: 应用接收过滤器设置
 *   步骤3: 如果过滤器设置成功，将模块标记为正常模式
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 * 返回值说明：无返回值
 */
void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    CO_ReturnError_t ret;

    if (CANmodule != NULL) {
        /* 步骤1: 先设置为非正常模式 */
        CANmodule->CANnormal = false;
        /* 步骤2: 应用接收过滤器 */
        ret = setRxFilters(CANmodule);
        if (ret == CO_ERROR_NO) {
            /* 步骤3: 将 CAN 模块切换到正常模式 */
            /* Put CAN module in normal mode */
            CANmodule->CANnormal = true;
        }
    }
}

/* 函数功能：初始化 CAN 模块，配置接收/发送缓冲区和 socketCAN 接口（非常重要的初始化函数）
 * 执行步骤：
 *   步骤1: 验证输入参数的有效性
 *   步骤2: 转换 CANptr 为 socketCAN 特定类型
 *   步骤3: 配置 CAN 模块的基本对象变量（epoll_fd、缓冲区数组等）
 *   步骤4: 初始化多接口模式下的 COB-ID 查找表（如果启用）
 *   步骤5: 分配并初始化 socketCAN 接收过滤器数组
 *   步骤6: 初始化所有接收缓冲区的默认值
 *   步骤7: 在单接口模式下添加 CAN 接口
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   CANptr - socketCAN 特定的指针，包含 epoll_fd 和接口索引
 *   rxArray - 接收缓冲区数组
 *   rxSize - 接收缓冲区数量
 *   txArray - 发送缓冲区数组
 *   txSize - 发送缓冲区数量
 *   CANbitRate - CAN 波特率（socketCAN 中未使用，由系统配置）
 * 返回值说明：
 *   CO_ERROR_NO - 初始化成功
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 *   CO_ERROR_OUT_OF_MEMORY - 内存分配失败
 */
CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    int32_t ret;
    uint16_t i;
    (void)CANbitRate; /* socketCAN 中波特率由系统配置，此参数未使用 */

    /* 步骤1: 验证参数有效性 */
    /* verify arguments */
    if (CANmodule == NULL || CANptr == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* 步骤2: 转换为 socketCAN 特定的指针类型 */
    CO_CANptrSocketCan_t* CANptrReal = (CO_CANptrSocketCan_t*)CANptr;

    /* 步骤3: 配置对象变量 - 初始化 CAN 模块的基本属性 */
    /* Configure object variables */
    CANmodule->epoll_fd = CANptrReal->epoll_fd; /* 复制 epoll 文件描述符，用于事件监听 */
    CANmodule->CANinterfaces = NULL;
    CANmodule->CANinterfaceCount = 0;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false; /* 初始状态为非正常模式 */
    CANmodule->CANtxCount = 0;

#if CO_DRIVER_MULTI_INTERFACE > 0
    /* 步骤4: 初始化多接口模式下的 COB-ID 到索引的查找表 */
    for (i = 0; i < CO_CAN_MSG_SFF_MAX_COB_ID; i++) {
        CANmodule->rxIdentToIndex[i] = CO_INVALID_COB_ID;
        CANmodule->txIdentToIndex[i] = CO_INVALID_COB_ID;
    }
#endif

    /* 步骤5: 初始化 socketCAN 过滤器。CAN 模块过滤器将通过 CO_CANrxBufferInit() 函数配置，
     * 该函数由各个 CANopen 初始化函数调用 */
    /* initialize socketCAN filters. CAN module filters will be configured with
     * CO_CANrxBufferInit() functions, called by separate CANopen init functions */
    CANmodule->rxFilter = calloc(CANmodule->rxSize, sizeof(struct can_filter));
    if (CANmodule->rxFilter == NULL) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "malloc()");
        return CO_ERROR_OUT_OF_MEMORY;
    }

    /* 步骤6: 初始化所有接收缓冲区为默认值 */
    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
        rxArray[i].can_ifindex = 0;
        rxArray[i].timestamp.tv_sec = 0;
        rxArray[i].timestamp.tv_nsec = 0;
    }

#if CO_DRIVER_MULTI_INTERFACE == 0
    /* 步骤7: 单接口模式下添加一个 CAN 接口 */
    /* add one interface */
    ret = CO_CANmodule_addInterface(CANmodule, CANptrReal->can_ifindex);
    if (ret != CO_ERROR_NO) {
        CO_CANmodule_disable(CANmodule);
        return ret;
    }
#endif
    return CO_ERROR_NO;
}

/* 函数功能：启用并添加一个 socketCAN 接口到 CAN 模块
 * 执行步骤：
 *   步骤1: 检查模块是否处于配置状态（非正常模式）
 *   步骤2: 扩展接口列表，添加新接口
 *   步骤3: 获取接口索引对应的接口名称
 *   步骤4: 创建原始 CAN socket
 *   步骤5: 启用 socket 接收队列溢出检测功能
 *   步骤6: 启用软件时间戳模式（硬件时间戳在某些设备上不能正常工作）
 *   步骤7: 获取并记录接收缓冲区大小
 *   步骤8: 绑定 socket 到指定的 CAN 接口
 *   步骤9: 初始化错误处理器并设置错误帧过滤器（如果启用错误报告）
 *   步骤10: 将 socket 添加到 epoll 事件监听
 *   步骤11: 初始禁用接收（通过调用 CO_CANsetNormalMode() 启动）
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   can_ifindex - CAN 网络接口索引号（如 can0 的索引）
 * 返回值说明：
 *   CO_ERROR_NO - 接口添加成功
 *   CO_ERROR_INVALID_STATE - 模块处于正常模式，无法修改配置
 *   CO_ERROR_OUT_OF_MEMORY - 内存分配失败
 *   CO_ERROR_ILLEGAL_ARGUMENT - 接口索引无效
 *   CO_ERROR_SYSCALL - 系统调用失败
 */
/* enable socketCAN */
#if CO_DRIVER_MULTI_INTERFACE == 0
static
#endif
    CO_ReturnError_t
    CO_CANmodule_addInterface(CO_CANmodule_t* CANmodule, int can_ifindex) {
    int32_t ret;
    int32_t tmp;
    int32_t bytes;
    char* ifName;
    socklen_t sLen;
    CO_CANinterface_t* interface;
    struct sockaddr_can sockAddr;
    struct epoll_event ev = {0};
#if CO_DRIVER_ERROR_REPORTING > 0
    can_err_mask_t err_mask;
#endif

    /* 步骤1: 检查是否可以修改配置 */
    if (CANmodule->CANnormal != false) {
        /* 现在无法更改配置！模块必须处于配置模式 */
        /* can't change config now! */
        return CO_ERROR_INVALID_STATE;
    }

    /* 步骤2: 将接口添加到接口列表 */
    /* Add interface to interface list */
    CANmodule->CANinterfaceCount++;
    CANmodule->CANinterfaces = realloc(CANmodule->CANinterfaces,
                                       ((CANmodule->CANinterfaceCount) * sizeof(*CANmodule->CANinterfaces)));
    if (CANmodule->CANinterfaces == NULL) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "malloc()");
        return CO_ERROR_OUT_OF_MEMORY;
    }
    interface = &CANmodule->CANinterfaces[CANmodule->CANinterfaceCount - 1];

    /* 步骤3: 根据接口索引获取接口名称（如 can0, can1） */
    interface->can_ifindex = can_ifindex;
    ifName = if_indextoname(can_ifindex, interface->ifName);
    if (ifName == NULL) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "if_indextoname()");
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* 步骤4: 创建原始 CAN socket */
    /* Create socket */
    interface->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (interface->fd < 0) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "socket(can)");
        return CO_ERROR_SYSCALL;
    }

    /* 步骤5: 启用 socket 接收队列溢出检测 */
    /* enable socket rx queue overflow detection */
    tmp = 1;
    ret = setsockopt(interface->fd, SOL_SOCKET, SO_RXQ_OVFL, &tmp, sizeof(tmp));
    if (ret < 0) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "setsockopt(ovfl)");
        return CO_ERROR_SYSCALL;
    }

    /* 步骤6: 启用软件时间戳模式（硬件时间戳在所有设备上不能正常工作）*/
    /* enable software time stamp mode (hardware timestamps do not work properly on all devices) */
    tmp = (SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE);
    ret = setsockopt(interface->fd, SOL_SOCKET, SO_TIMESTAMPING, &tmp, sizeof(tmp));
    if (ret < 0) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "setsockopt(timestamping)");
        return CO_ERROR_SYSCALL;
    }

    // todo - 修改接收缓冲区大小？第一个需要 root 权限
    // todo - modify rx buffer size? first one needs root
    // ret = setsockopt(fd, SOL_SOCKET, SO_RCVBUFFORCE, (void *)&bytes, sLen);
    // ret = setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (void *)&bytes, sLen);

    /* 步骤7: 打印 socket 接收缓冲区大小（根据经验，内核为每条 CAN 消息预留约 450 字节）*/
    /* print socket rx buffer size in bytes (In my experience, the kernel reserves
     * around 450 bytes for each CAN message) */
    sLen = sizeof(bytes);
    getsockopt(interface->fd, SOL_SOCKET, SO_RCVBUF, (void*)&bytes, &sLen);
    if (sLen == sizeof(bytes)) {
        log_printf(LOG_INFO, CAN_SOCKET_BUF_SIZE, interface->ifName, bytes / 446, bytes);
    }

    /* 步骤8: 绑定 socket 到指定的 CAN 接口 */
    /* bind socket */
    memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.can_family = AF_CAN;
    sockAddr.can_ifindex = can_ifindex;
    ret = bind(interface->fd, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
    if (ret < 0) {
        log_printf(LOG_ERR, CAN_BINDING_FAILED, interface->ifName);
        log_printf(LOG_DEBUG, DBG_ERRNO, "bind()");
        return CO_ERROR_SYSCALL;
    }

#if CO_DRIVER_ERROR_REPORTING > 0
    /* 步骤9: 初始化错误处理器并设置错误帧生成 */
    CO_CANerror_init(&interface->errorhandler, interface->fd, interface->ifName);
    /* 设置错误帧生成。实际可用的功能取决于 CAN 内核驱动 */
    /* set up error frame generation. What actually is available depends on your CAN kernel driver */
#ifdef DEBUG
    err_mask = CAN_ERR_MASK; /* 调试模式下启用所有错误帧 */
    // enable ALL error frames
#else
    err_mask = CAN_ERR_ACK | CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR; /* 启用关键错误帧 */
#endif
    ret = setsockopt(interface->fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
    if (ret < 0) {
        log_printf(LOG_ERR, CAN_ERROR_FILTER_FAILED, interface->ifName);
        log_printf(LOG_DEBUG, DBG_ERRNO, "setsockopt(can err)");
        return CO_ERROR_SYSCALL;
    }
#endif /* CO_DRIVER_ERROR_REPORTING */

    /* 步骤10: 将 socket 添加到 epoll 事件监听 */
    /* Add socket to epoll */
    ev.events = EPOLLIN; /* 监听可读事件 */
    ev.data.fd = interface->fd;
    ret = epoll_ctl(CANmodule->epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);
    if (ret < 0) {
        log_printf(LOG_DEBUG, DBG_ERRNO, "epoll_ctl(can)");
        return CO_ERROR_SYSCALL;
    }

    /* 步骤11: 接收功能通过调用 #CO_CANsetNormalMode() 启动 */
    /* rx is started by calling #CO_CANsetNormalMode() */
    ret = disableRx(CANmodule);

    return ret;
}

/* 函数功能：禁用 CAN 模块，清理所有接口和资源
 * 执行步骤：
 *   步骤1: 检查模块指针有效性
 *   步骤2: 将模块设置为非正常模式
 *   步骤3: 遍历并清理所有 CAN 接口
 *   步骤4: 禁用每个接口的错误处理器（如果启用）
 *   步骤5: 从 epoll 中移除 socket 并关闭文件描述符
 *   步骤6: 释放接口列表内存
 *   步骤7: 释放接收过滤器内存
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 * 返回值说明：无返回值
 */
void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    uint32_t i;

    /* 步骤1: 检查指针有效性 */
    if (CANmodule == NULL) {
        return;
    }

    /* 步骤2: 设置为非正常模式 */
    CANmodule->CANnormal = false;

    /* 步骤3: 清理所有接口 */
    /* clear interfaces */
    for (i = 0; i < CANmodule->CANinterfaceCount; i++) {
        CO_CANinterface_t* interface = &CANmodule->CANinterfaces[i];

#if CO_DRIVER_ERROR_REPORTING > 0
        /* 步骤4: 禁用错误处理器 */
        CO_CANerror_disable(&interface->errorhandler);
#endif

        /* 步骤5: 从 epoll 移除并关闭 socket */
        epoll_ctl(CANmodule->epoll_fd, EPOLL_CTL_DEL, interface->fd, NULL);
        close(interface->fd);
        interface->fd = -1;
    }
    /* 步骤6: 重置接口计数并释放接口列表内存 */
    CANmodule->CANinterfaceCount = 0;
    if (CANmodule->CANinterfaces != NULL) {
        free(CANmodule->CANinterfaces);
    }
    CANmodule->CANinterfaces = NULL;

    /* 步骤7: 释放接收过滤器内存 */
    if (CANmodule->rxFilter != NULL) {
        free(CANmodule->rxFilter);
    }
    CANmodule->rxFilter = NULL;
}

/* 函数功能：初始化 CAN 接收缓冲区，配置过滤器和回调函数
 * 执行步骤：
 *   步骤1: 验证模块和索引有效性
 *   步骤2: 获取指定索引的接收缓冲区
 *   步骤3: 在多接口模式下更新 COB-ID 到索引的映射
 *   步骤4: 配置缓冲区的对象和回调函数
 *   步骤5: 设置 CAN 标识符和掩码（处理 RTR 位）
 *   步骤6: 配置 socketCAN 接收过滤器
 *   步骤7: 如果模块处于正常模式，立即应用过滤器
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   index - 接收缓冲区索引
 *   ident - CAN 标识符（COB-ID）
 *   mask - CAN 掩码，用于过滤匹配
 *   rtr - 是否为远程传输请求帧
 *   object - 关联的对象指针，传递给回调函数
 *   CANrx_callback - 接收到匹配消息时的回调函数
 * 返回值说明：
 *   CO_ERROR_NO - 初始化成功
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 */
CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    /* 步骤1: 验证参数 */
    if ((CANmodule != NULL) && (index < CANmodule->rxSize)) {
        CO_CANrx_t* buffer;

        /* 步骤2: 获取将要配置的缓冲区 */
        /* buffer, which will be configured */
        buffer = &CANmodule->rxArray[index];

#if CO_DRIVER_MULTI_INTERFACE > 0
        /* 步骤3: 更新 COB-ID 到索引的映射表 */
        CO_CANsetIdentToIndex(CANmodule->rxIdentToIndex, index, ident, buffer->ident);
#endif

        /* 步骤4: 配置对象变量 */
        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;
        buffer->can_ifindex = 0;
        buffer->timestamp.tv_nsec = 0;
        buffer->timestamp.tv_sec = 0;

        /* 步骤5: 设置 CAN 标识符和掩码，与 CAN 模块位对齐 */
        /* CAN identifier and CAN mask, bit aligned with CAN module */
        buffer->ident = ident & CAN_SFF_MASK; /* 仅保留标准帧 ID 位 */
        if (rtr) {
            buffer->ident |= CAN_RTR_FLAG; /* 设置 RTR 标志位 */
        }
        buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG; /* 掩码包含扩展帧和 RTR 标志 */

        /* 步骤6: 设置 CAN 硬件模块过滤器和掩码 */
        /* Set CAN hardware module filter and mask. */
        CANmodule->rxFilter[index].can_id = buffer->ident;
        CANmodule->rxFilter[index].can_mask = buffer->mask;
        /* 步骤7: 如果处于正常模式，立即应用过滤器 */
        if (CANmodule->CANnormal) {
            ret = setRxFilters(CANmodule);
        }
    } else {
        log_printf(LOG_DEBUG, DBG_CAN_RX_PARAM_FAILED, "illegal argument");
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

#if CO_DRIVER_MULTI_INTERFACE > 0

/* 函数功能：获取接收缓冲区关联的接口信息
 * 执行步骤：
 *   步骤1: 验证模块有效性
 *   步骤2: 根据 COB-ID 查找对应的缓冲区索引
 *   步骤3: 验证索引有效性
 *   步骤4: 返回接口索引和时间戳信息
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   ident - CAN 标识符（COB-ID）
 *   can_ifindexRx - 接收接口索引的输出参数（可选）
 *   timestamp - 时间戳的输出参数（可选）
 * 返回值说明：
 *   true - 找到有效接口
 *   false - 未找到或参数无效
 */
bool_t
CO_CANrxBuffer_getInterface(CO_CANmodule_t* CANmodule, uint16_t ident, int* can_ifindexRx, struct timespec* timestamp) {
    CO_CANrx_t* buffer;

    /* 步骤1: 验证模块 */
    if (CANmodule == NULL) {
        return false;
    }

    /* 步骤2: 根据标识符查找索引 */
    const uint32_t index = CO_CANgetIndexFromIdent(CANmodule->rxIdentToIndex, ident);
    /* 步骤3: 验证索引有效性 */
    if ((index == CO_INVALID_COB_ID) || (index > CANmodule->rxSize)) {
        return false;
    }
    buffer = &CANmodule->rxArray[index];

    /* 步骤4: 返回值 */
    /* return values */
    if (can_ifindexRx != NULL) {
        *can_ifindexRx = buffer->can_ifindex;
    }
    if (timestamp != NULL) {
        *timestamp = buffer->timestamp;
    }
    if (buffer->can_ifindex != 0) {
        return true;
    } else {
        return false;
    }
}

#endif /* CO_DRIVER_MULTI_INTERFACE */

/* 函数功能：初始化 CAN 发送缓冲区，配置标识符和数据长度
 * 执行步骤：
 *   步骤1: 验证模块和索引有效性
 *   步骤2: 获取指定索引的发送缓冲区
 *   步骤3: 在多接口模式下更新 COB-ID 到索引的映射
 *   步骤4: 初始化接口索引为 0（表示所有接口）
 *   步骤5: 设置 CAN 标识符（处理 RTR 位）
 *   步骤6: 配置数据长度、缓冲区状态和同步标志
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   index - 发送缓冲区索引
 *   ident - CAN 标识符（COB-ID）
 *   rtr - 是否为远程传输请求帧
 *   noOfBytes - 数据字节数（DLC）
 *   syncFlag - 是否为同步消息
 * 返回值说明：
 *   返回配置好的发送缓冲区指针，失败返回 NULL
 */
CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    /* 步骤1: 验证参数 */
    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* 步骤2: 获取指定的发送缓冲区 */
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

#if CO_DRIVER_MULTI_INTERFACE > 0
        /* 步骤3: 更新发送缓冲区的 COB-ID 映射 */
        CO_CANsetIdentToIndex(CANmodule->txIdentToIndex, index, ident, buffer->ident);
#endif

        /* 步骤4: 初始化接口索引（0 表示所有接口）*/
        buffer->can_ifindex = 0;

        /* 步骤5: 设置 CAN 标识符和 RTR 标志 */
        /* CAN identifier and rtr */
        buffer->ident = ident & CAN_SFF_MASK; /* 仅保留标准帧 ID 位 */
        if (rtr) {
            buffer->ident |= CAN_RTR_FLAG; /* 设置 RTR 标志位 */
        }
        /* 步骤6: 配置数据长度和状态标志 */
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false; /* 缓冲区初始为空 */
        buffer->syncFlag = syncFlag; /* 同步消息标志 */
    }

    return buffer;
}

#if CO_DRIVER_MULTI_INTERFACE > 0

/* 函数功能：为发送缓冲区设置指定的发送接口
 * 执行步骤：
 *   步骤1: 验证模块有效性
 *   步骤2: 根据 COB-ID 查找发送缓冲区索引
 *   步骤3: 验证索引有效性
 *   步骤4: 设置发送接口索引
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   ident - CAN 标识符（COB-ID）
 *   can_ifindexTx - 要使用的发送接口索引
 * 返回值说明：
 *   CO_ERROR_NO - 设置成功
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 */
CO_ReturnError_t
CO_CANtxBuffer_setInterface(CO_CANmodule_t* CANmodule, uint16_t ident, int can_ifindexTx) {
    if (CANmodule != NULL) {
        uint32_t index;

        /* 步骤2: 查找缓冲区索引 */
        index = CO_CANgetIndexFromIdent(CANmodule->txIdentToIndex, ident);
        /* 步骤3: 验证索引 */
        if ((index == CO_INVALID_COB_ID) || (index > CANmodule->txSize)) {
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }
        /* 步骤4: 设置接口索引 */
        CANmodule->txArray[index].can_ifindex = can_ifindexTx;

        return CO_ERROR_NO;
    }
    return CO_ERROR_ILLEGAL_ARGUMENT;
}

#endif /* CO_DRIVER_MULTI_INTERFACE */
#if CO_DRIVER_MULTI_INTERFACE > 0

/* 函数功能：通过指定接口发送 CAN 消息（多接口模式内部函数）
 * 执行步骤：
 *   步骤1: 验证参数和接口有效性
 *   步骤2: 检查接口状态（如果启用错误报告）
 *   步骤3: 循环尝试发送，处理不同的错误情况：
 *         - EINTR: 被中断，重试
 *         - EAGAIN: 队列满，退出循环
 *         - ENOBUFS: 缓冲区满，返回忙碌
 *   步骤4: 验证发送结果，记录错误
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   buffer - 发送缓冲区指针
 *   interface - 要使用的 CAN 接口指针
 * 返回值说明：
 *   CO_ERROR_NO - 发送成功
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 *   CO_ERROR_INVALID_STATE - 接口状态无效
 *   CO_ERROR_TX_BUSY - 发送忙碌
 *   CO_ERROR_TX_OVERFLOW - 发送溢出
 * 注意：socketCAN 不支持阻塞写入。在 ENOBUFS 错误时可等待几百微秒后重试
 */
/* send CAN message */
static CO_ReturnError_t
CO_CANCheckSendInterface(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer, CO_CANinterface_t* interface) {
    CO_ReturnError_t err = CO_ERROR_NO;
#if CO_DRIVER_ERROR_REPORTING > 0
    CO_CANinterfaceState_t ifState;
#endif
    ssize_t n;

    /* 步骤1: 验证参数 */
    if (CANmodule == NULL || interface == NULL || interface->fd < 0) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

#if CO_DRIVER_ERROR_REPORTING > 0
    /* 步骤2: 检查接口状态 */
    ifState = CO_CANerror_txMsg(&interface->errorhandler);
    switch (ifState) {
        case CO_INTERFACE_ACTIVE:
            /* 接口活跃，继续发送 */
            /* continue */
            break;
        case CO_INTERFACE_LISTEN_ONLY:
            /* 静默模式，丢弃消息 */
            /* silently drop message */
            return CO_ERROR_NO;
        default: return CO_ERROR_INVALID_STATE;
    }
#endif

    /* 步骤3: 尝试发送消息，处理各种错误情况 */
    do {
        errno = 0;
        n = send(interface->fd, buffer, CAN_MTU, MSG_DONTWAIT);
        if (errno == EINTR) {
            /* 被中断，重试 */
            /* try again */
            continue;
        } else if (errno == EAGAIN) {
            /* socket 队列满 */
            /* socket queue full */
            break;
        } else if (errno == ENOBUFS) {
            /* socketCAN 不支持阻塞写入。可以在此等待几百微秒后重试 */
            /* socketCAN doesn't support blocking write. You can wait here for
             * a few hundred us and then try again */
#if CO_DRIVER_ERROR_REPORTING > 0
            interface->errorhandler.CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
#endif
            return CO_ERROR_TX_BUSY;
        } else if (n != CAN_MTU) {
            break; /* 其他错误 */
        }
    } while (errno != 0);

    /* 步骤4: 检查发送结果 */
    if (n != CAN_MTU) {
#if CO_DRIVER_ERROR_REPORTING > 0
        interface->errorhandler.CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
#endif
        log_printf(LOG_ERR, DBG_CAN_TX_FAILED, buffer->ident, interface->ifName);
        log_printf(LOG_DEBUG, DBG_ERRNO, "send()");
        err = CO_ERROR_TX_OVERFLOW;
    }

    return err;
}

/*
 * 函数功能：与 #CO_CANsend() 类似，但确保驱动中为更重要的消息保留足够空间
 * 说明：默认阈值为 50%，或至少 1 个消息缓冲区。如果发送违反这些限制，
 *       将返回 #CO_ERROR_TX_OVERFLOW，消息不会被发送。
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   buffer - 发送缓冲区指针，由 CO_CANtxBufferInit() 返回。
 *            在调用函数前必须写入数据字节。
 * 返回值说明：
 *   CO_ERROR_NO - 发送成功
 *   CO_ERROR_TX_OVERFLOW - 发送溢出
 *   CO_ERROR_TX_BUSY - 发送忙碌
 *   CO_ERROR_TX_PDO_WINDOW - 同步 TPDO 超出窗口
 * 注意：此函数不在头文件中，因为 CO_CANCheckSend() 的使用尚不明确
 */
/*
 * The same as #CO_CANsend(), but ensures that there is enough space remaining
 * in the driver for more important messages.
 *
 * The default threshold is 50%, or at least 1 message buffer. If sending
 * would violate those limits, #CO_ERROR_TX_OVERFLOW is returned and the
 * message will not be sent.
 *
 * (It is not in header, because usage of CO_CANCheckSend() is unclear.)
 *
 * @param CANmodule This object.
 * @param buffer Pointer to transmit buffer, returned by CO_CANtxBufferInit().
 * Data bytes must be written in buffer before function call.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_TX_OVERFLOW, CO_ERROR_TX_BUSY or
 * CO_ERROR_TX_PDO_WINDOW (Synchronous TPDO is outside window).
 */
CO_ReturnError_t CO_CANCheckSend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer);

/* 函数功能：发送 CAN 消息（多接口模式）
 * 说明：调用 CO_CANCheckSend()，如果返回忙碌则转换为溢出错误
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   buffer - 发送缓冲区指针
 * 返回值说明：
 *   CO_ERROR_NO - 发送成功
 *   CO_ERROR_TX_OVERFLOW - 发送溢出
 */
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err;
    err = CO_CANCheckSend(CANmodule, buffer);
    if (err == CO_ERROR_TX_BUSY) {
        /* 发送操作没有"忙碌"状态，转换为溢出 */
        /* send doesn't have "busy" */
        log_printf(LOG_ERR, DBG_CAN_TX_FAILED, buffer->ident, "CANx");
        log_printf(LOG_DEBUG, DBG_ERRNO, "send()");
        err = CO_ERROR_TX_OVERFLOW;
    }
    return err;
}

/* 函数功能：检查并发送 CAN 消息到合适的接口（多接口模式）
 * 执行步骤：
 *   步骤1: 遍历所有 CAN 接口
 *   步骤2: 检查接口是否匹配（接口索引为 0 或匹配指定接口）
 *   步骤3: 在匹配的接口上发送消息
 *   步骤4: 记录并返回最后一个错误
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   buffer - 发送缓冲区指针
 * 返回值说明：
 *   CO_ERROR_NO - 所有接口发送成功
 *   其他 - 返回最后一个接口的错误代码
 */
CO_ReturnError_t
CO_CANCheckSend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    uint32_t i;
    CO_ReturnError_t err = CO_ERROR_NO;

    /* 步骤1: 检查应在哪些接口上发送此消息 */
    /* check on which interfaces to send this messages */
    for (i = 0; i < CANmodule->CANinterfaceCount; i++) {
        CO_CANinterface_t* interface = &CANmodule->CANinterfaces[i];

        /* 步骤2: 检查接口匹配（0 表示所有接口）*/
        if ((buffer->can_ifindex == 0) || buffer->can_ifindex == interface->can_ifindex) {

            CO_ReturnError_t tmp;

            /* 步骤3: 匹配，使用此接口发送 */
            /* match, use this one */
            tmp = CO_CANCheckSendInterface(CANmodule, buffer, interface);
            if (tmp) {
                /* 步骤4: 仅将最后一个错误返回给调用者 */
                /* only last error is returned to callee */
                err = tmp;
            }
        }
    }

    return err;
}

/* 警告：CO_CANsend() 对于 CO_DRIVER_MULTI_INTERFACE > 0 模式已过时 */
#warning CO_CANsend() is outdated for CO_DRIVER_MULTI_INTERFACE > 0

#endif /* CO_DRIVER_MULTI_INTERFACE > 0 */

#if CO_DRIVER_MULTI_INTERFACE == 0

/* 函数功能：发送 CAN 消息（单接口模式）
 * 执行步骤：
 *   步骤1: 验证参数有效性
 *   步骤2: 获取第一个 CAN 接口
 *   步骤3: 检查发送缓冲区是否已满（溢出检测）
 *   步骤4: 调用 send() 尝试发送消息
 *   步骤5: 根据返回值处理不同情况：
 *         - 成功：清除 bufferFull 标志
 *         - 忙碌：设置 bufferFull 标志，由 CO_CANmodule_process() 重发
 *         - 错误：记录错误状态
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   buffer - 发送缓冲区指针，包含要发送的数据
 * 返回值说明：
 *   CO_ERROR_NO - 发送成功
 *   CO_ERROR_TX_OVERFLOW - 缓冲区溢出
 *   CO_ERROR_TX_BUSY - 发送忙碌，稍后重试
 *   CO_ERROR_SYSCALL - 系统调用失败
 *   CO_ERROR_ILLEGAL_ARGUMENT - 参数无效
 * 注意：使用 CO_CANtx_t->bufferFull 标志处理发送缓冲区满的情况，
 *       未发送的消息将在 CO_CANmodule_process() 中重新发送
 */
/* Change handling of tx buffer full in CO_CANsend(). Use CO_CANtx_t->bufferFull
 * flag. Re-transmit undelivered message inside CO_CANmodule_process(). */
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* 步骤1: 验证参数 */
    if (CANmodule == NULL || buffer == NULL || CANmodule->CANinterfaceCount == 0) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* 步骤2: 获取第一个（唯一）CAN 接口 */
    CO_CANinterface_t* interface = &CANmodule->CANinterfaces[0];
    if (interface == NULL || interface->fd < 0) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* 步骤3: 验证是否发生溢出（缓冲区已满）*/
    /* Verify overflow */
    if (buffer->bufferFull) {
#if CO_DRIVER_ERROR_REPORTING > 0
        interface->errorhandler.CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
#endif
        log_printf(LOG_ERR, DBG_CAN_TX_FAILED, buffer->ident, interface->ifName);
        err = CO_ERROR_TX_OVERFLOW;
    }

    /* 步骤4: 尝试发送消息（非阻塞模式）*/
    errno = 0;
    ssize_t n = send(interface->fd, buffer, CAN_MTU, MSG_DONTWAIT);
    /* 步骤5: 处理发送结果 */
    if (errno == 0 && n == CAN_MTU) {
        /* 发送成功 */
        /* success */
        if (buffer->bufferFull) {
            buffer->bufferFull = false;
            CANmodule->CANtxCount--;
        }
    } else if (errno == EINTR || errno == EAGAIN || errno == ENOBUFS) {
        /* 发送失败，消息将由 CO_CANmodule_process() 重新发送 */
        /* Send failed, message will be re-sent by CO_CANmodule_process() */
        if (!buffer->bufferFull) {
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
        err = CO_ERROR_TX_BUSY;
    } else {
        /* 未知错误 */
        /* Unknown error */
        log_printf(LOG_DEBUG, DBG_ERRNO, "send()");
#if CO_DRIVER_ERROR_REPORTING > 0
        interface->errorhandler.CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
#endif
        err = CO_ERROR_SYSCALL;
    }

    return err;
}

#endif /* CO_DRIVER_MULTI_INTERFACE == 0 */

/* 函数功能：清除待处理的同步 PDO 消息（socketCAN 中为空实现）
 * 说明：在 socketCAN 中，消息要么被写入 socket 队列，要么被丢弃，无需手动清除
 * 参数说明：
 *   CANmodule - CAN 模块对象指针（未使用）
 * 返回值说明：无返回值
 */
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    /* 消息要么写入 socket 队列，要么被丢弃 */
    /* Messages are either written to the socket queue or dropped */
}

/* 函数功能：处理 CAN 模块，更新错误状态并重发未发送的消息
 * 执行步骤：
 *   步骤1: 验证模块有效性
 *   步骤2: 更新 CAN 错误状态（如果启用错误报告）
 *   步骤3: 在单接口模式下，重发之前未成功发送的消息
 *   步骤4: 查找标记为 bufferFull 的发送缓冲区
 *   步骤5: 尝试重新发送该消息
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 * 返回值说明：无返回值
 * 注意：socketCAN 不支持类似微控制器的错误计数器，错误信息通过特殊的 CAN 消息传递
 */
void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    /* 步骤1: 验证模块 */
    if (CANmodule == NULL || CANmodule->CANinterfaceCount == 0) {
        return;
    }

#if CO_DRIVER_ERROR_REPORTING > 0
    /* 步骤2: 更新错误状态 - socketCAN 不支持类似微控制器的错误计数器。
     * 如果发生错误，驱动程序会创建特殊的 CAN 消息，并像常规消息一样被应用程序接收。
     * 因此，错误计数器评估包含在接收函数中。
     * 这里我们只从第一个 CAN 接口复制评估后的 CANerrorStatus。 */
    /* socketCAN doesn't support microcontroller-like error counters. If an
     * error has occured, a special can message is created by the driver and
     * received by the application like a regular message.
     * Therefore, error counter evaluation is included in rx function.
     * Here we just copy evaluated CANerrorStatus from the first CAN interface. */

    CANmodule->CANerrorStatus = CANmodule->CANinterfaces[0].errorhandler.CANerrorStatus;
#endif

#if CO_DRIVER_MULTI_INTERFACE == 0
    /* 步骤3: 如果之前有消息未发送，重新调用 CO_CANsend() */
    /* recall CO_CANsend(), if message was unsent before */
    if (CANmodule->CANtxCount > 0) {
        bool_t found = false;

        /* 步骤4: 遍历发送缓冲区，查找未发送的消息 */
        for (uint16_t i = 0; i < CANmodule->txSize; i++) {
            CO_CANtx_t* buffer = &CANmodule->txArray[i];

            if (buffer->bufferFull) {
                /* 步骤5: 清除标志并尝试重新发送 */
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;
                CO_CANsend(CANmodule, buffer);
                found = true;
                break; /* 每次处理一条消息 */
            }
        }

        /* 如果没找到，重置计数器（防止计数错误）*/
        if (!found) {
            CANmodule->CANtxCount = 0;
        }
    }
#endif /* CO_DRIVER_MULTI_INTERFACE == 0 */
}

/* 函数功能：从 socket 读取 CAN 消息并验证错误
 * 执行步骤：
 *   步骤1: 准备 recvmsg() 所需的数据结构
 *   步骤2: 设置 IO 向量指向消息缓冲区
 *   步骤3: 设置消息头以接收控制信息（时间戳和溢出计数）
 *   步骤4: 调用 recvmsg() 从 socket 读取消息
 *   步骤5: 验证接收的数据大小是否正确
 *   步骤6: 遍历控制消息，提取时间戳和队列溢出信息
 *   步骤7: 处理接收队列溢出情况
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   interface - CAN 接口指针
 *   msg - CAN 消息结构指针（输出参数）
 *   timestamp - 消息时间戳指针（输出参数）
 * 返回值说明：
 *   CO_ERROR_NO - 读取成功
 *   CO_ERROR_SYSCALL - 系统调用失败
 * 注意：使用 recvmsg() 而非 read()，以获取 socket 的统计信息
 */
/* Read CAN message from socket and verify some errors */
static CO_ReturnError_t
CO_CANread(CO_CANmodule_t* CANmodule, CO_CANinterface_t* interface,
           struct can_frame* msg,      /* CAN 消息，返回值 - CAN message, return value */
           struct timespec* timestamp) /* CAN 消息的时间戳，返回值 - timestamp of CAN message, return value */
{
    int32_t n;
    uint32_t dropped;
    /* 步骤1: recvmsg - 类似 read，但可以生成 socket 的统计信息（参考 berlios candump.c）*/
    /* recvmsg - like read, but generates statistics about the socket example in berlios candump.c */
    struct iovec iov;
    struct msghdr msghdr;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(dropped))];
    struct cmsghdr* cmsg;

    /* 步骤2: 设置 IO 向量指向消息缓冲区 */
    iov.iov_base = msg;
    iov.iov_len = sizeof(*msg);

    /* 步骤3: 配置消息头结构以接收辅助数据 */
    msghdr.msg_name = NULL;
    msghdr.msg_namelen = 0;
    msghdr.msg_iov = &iov;
    msghdr.msg_iovlen = 1;
    msghdr.msg_control = &ctrlmsg; /* 用于接收控制消息（时间戳等）*/
    msghdr.msg_controllen = sizeof(ctrlmsg);
    msghdr.msg_flags = 0;

    /* 步骤4: 从 socket 接收消息 */
    n = recvmsg(interface->fd, &msghdr, 0);
    /* 步骤5: 检查接收的数据大小 */
    if (n != CAN_MTU) {
#if CO_DRIVER_ERROR_REPORTING > 0
        interface->errorhandler.CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
#endif
        log_printf(LOG_DEBUG, DBG_CAN_RX_FAILED, interface->ifName);
        log_printf(LOG_DEBUG, DBG_ERRNO, "recvmsg()");
        return CO_ERROR_SYSCALL;
    }

    /* 步骤6: 检查接收队列溢出，获取接收时间 */
    /* check for rx queue overflow, get rx time */
    for (cmsg = CMSG_FIRSTHDR(&msghdr); cmsg && (cmsg->cmsg_level == SOL_SOCKET); cmsg = CMSG_NXTHDR(&msghdr, cmsg)) {
        if (cmsg->cmsg_type == SO_TIMESTAMPING) {
            /* 这是系统时间，不是单调时间！*/
            /* this is system time, not monotonic time! */
            *timestamp = ((struct timespec*)CMSG_DATA(cmsg))[0];
        } else if (cmsg->cmsg_type == SO_RXQ_OVFL) {
            /* 步骤7: 处理接收队列溢出 */
            dropped = *(uint32_t*)CMSG_DATA(cmsg);
            if (dropped > CANmodule->rxDropCount) {
#if CO_DRIVER_ERROR_REPORTING > 0
                interface->errorhandler.CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
#endif
                log_printf(LOG_ERR, CAN_RX_SOCKET_QUEUE_OVERFLOW, interface->ifName, dropped);
            }
            CANmodule->rxDropCount = dropped;
            // todo 使用此信息！- use this info!
        }
    }

    return CO_ERROR_NO;
}

/* 函数功能：在接收数组中查找匹配的消息并调用相应的回调函数
 * 执行步骤：
 *   步骤1: 将 socketCAN 消息转换为 CANopenNode 消息格式（二进制兼容）
 *   步骤2: 遍历接收缓冲区数组，查找匹配的 CAN-ID
 *   步骤3: 使用标识符和掩码进行匹配检查
 *   步骤4: 如果找到匹配，调用注册的回调函数处理消息
 *   步骤5: 可选地将消息复制到提供的缓冲区
 *   步骤6: 返回匹配的缓冲区索引或 -1
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   msg - 接收到的 CAN 消息（输入）
 *   buffer - 消息缓冲区指针，如果不为 NULL，消息将被复制到此缓冲区
 * 返回值说明：
 *   返回接收消息在 rxArray 中的索引，如果未找到匹配则返回 -1
 */
/* find msg inside rxArray and call corresponding CANrx_callback */
static int32_t
CO_CANrxMsg(                                                  /* 返回接收消息在 rxArray 中的索引或 -1 - return index of received message in rxArray or -1 */
            CO_CANmodule_t* CANmodule, struct can_frame* msg, /* CAN 消息输入 - CAN message input */
            CO_CANrxMsg_t* buffer)                            /* 如果不为 NULL，消息将被复制到缓冲区 - If not NULL, msg will be copied to buffer */
{
    int32_t retval;
    const CO_CANrxMsg_t* rcvMsg;  /* CAN 模块中接收消息的指针 - pointer to received message in CAN module */
    uint16_t index;               /* 接收消息的索引 - index of received message */
    CO_CANrx_t* rcvMsgObj = NULL; /* CO_CANmodule_t 对象中的接收消息对象 - receive message object from CO_CANmodule_t object. */
    bool_t msgMatched = false;

    /* 步骤1: CANopenNode CAN 消息与 socketCAN 消息二进制兼容，包括扩展标志 */
    /* CANopenNode can message is binary compatible to the socketCAN one, including the extension flags */
    // msg->can_id &= CAN_EFF_MASK;
    rcvMsg = (CO_CANrxMsg_t*)msg;

    /* 步骤2: 已接收到消息。在 CANmodule 的 rxArray 中搜索相同的 CAN-ID */
    /* Message has been received. Search rxArray from CANmodule for the
     * same CAN-ID. */
    rcvMsgObj = &CANmodule->rxArray[0];
    /* 步骤3: 遍历接收数组，使用掩码匹配标识符 */
    for (index = 0; index < CANmodule->rxSize; index++) {
        if (((rcvMsg->ident ^ rcvMsgObj->ident) & rcvMsgObj->mask) == 0U) {
            msgMatched = true;
            break; /* 找到匹配 */
        }
        rcvMsgObj++;
    }
    if (msgMatched) {
        /* 步骤4: 调用特定函数处理消息 */
        /* Call specific function, which will process the message */
        if ((rcvMsgObj != NULL) && (rcvMsgObj->CANrx_callback != NULL)) {
            rcvMsgObj->CANrx_callback(rcvMsgObj->object, (void*)rcvMsg);
        }
        /* 步骤5: 返回消息（可选复制到缓冲区）*/
        /* return message */
        if (buffer != NULL) {
            memcpy(buffer, rcvMsg, sizeof(*buffer));
        }
        /* 步骤6: 返回匹配的索引 */
        retval = index;
    } else {
        /* 未找到匹配 */
        retval = -1;
    }

    return retval;
}

/* 函数功能：从 epoll 事件处理 CAN 消息接收
 * 执行步骤：
 *   步骤1: 验证参数和模块状态
 *   步骤2: 遍历所有 CAN 接口，查找匹配的文件描述符
 *   步骤3: 处理不同类型的 epoll 事件：
 *         - EPOLLERR/EPOLLHUP: socket 错误或关闭
 *         - EPOLLIN: 有数据可读
 *   步骤4: 读取 CAN 消息和时间戳
 *   步骤5: 区分错误帧和数据帧：
 *         - 错误帧：调用错误处理函数
 *         - 数据帧：调用消息处理函数
 *   步骤6: 存储接收到的消息信息（时间戳和接口索引）
 * 参数说明：
 *   CANmodule - CAN 模块对象指针
 *   ev - epoll 事件结构指针
 *   buffer - 消息缓冲区指针（可选）
 *   msgIndex - 接收消息索引的输出参数（可选）
 * 返回值说明：
 *   true - 事件已处理（来自 CAN 接口）
 *   false - 事件未处理（不是 CAN 接口事件）
 */
bool_t
CO_CANrxFromEpoll(CO_CANmodule_t* CANmodule, struct epoll_event* ev, CO_CANrxMsg_t* buffer, int32_t* msgIndex) {
    /* 步骤1: 验证参数 */
    if (CANmodule == NULL || ev == NULL || CANmodule->CANinterfaceCount == 0) {
        return false;
    }

    /* 步骤2: 验证 CAN socket 上的 epoll 事件 */
    /* Verify for epoll events in CAN socket */
    for (uint32_t i = 0; i < CANmodule->CANinterfaceCount; i++) {
        CO_CANinterface_t* interface = &CANmodule->CANinterfaces[i];

        if (ev->data.fd == interface->fd) {
            /* 步骤3: 处理 epoll 事件 */
            if ((ev->events & (EPOLLERR | EPOLLHUP)) != 0) {
                /* 错误或挂起事件 */
                struct can_frame msg;
                /* epoll 检测到 socket 关闭/错误。尝试拉取事件 */
                /* epoll detected close/error on socket. Try to pull event */
                errno = 0;
                recv(ev->data.fd, &msg, sizeof(msg), MSG_DONTWAIT);
                log_printf(LOG_DEBUG, DBG_CAN_RX_EPOLL, ev->events, strerror(errno));
            } else if ((ev->events & EPOLLIN) != 0) {
                /* 可读事件 - 有新消息到达 */
                struct can_frame msg;
                struct timespec timestamp;

                /* 步骤4: 获取消息 */
                /* get message */
                CO_ReturnError_t err = CO_CANread(CANmodule, interface, &msg, &timestamp);

                if (err == CO_ERROR_NO && CANmodule->CANnormal) {

                    /* 步骤5: 区分错误帧和数据帧 */
                    if (msg.can_id & CAN_ERR_FLAG) {
                        /* 错误消息 */
                        /* error msg */
#if CO_DRIVER_ERROR_REPORTING > 0
                        CO_CANerror_rxMsgError(&interface->errorhandler, &msg);
#endif
                    } else {
                        /* 数据消息 */
                        /* data msg */
#if CO_DRIVER_ERROR_REPORTING > 0
                        /* 必要时清除 listenOnly 和 noackCounter */
                        /* clear listenOnly and noackCounter if necessary */
                        CO_CANerror_rxMsg(&interface->errorhandler);
#endif
                        /* 处理接收到的数据消息 */
                        int32_t idx = CO_CANrxMsg(CANmodule, &msg, buffer);
                        if (idx > -1) {
                            /* 步骤6: 存储消息信息 */
                            /* Store message info */
                            CANmodule->rxArray[idx].timestamp = timestamp;
                            CANmodule->rxArray[idx].can_ifindex = interface->can_ifindex;
                        }
                        if (msgIndex != NULL) {
                            *msgIndex = idx;
                        }
                    }
                }
            } else {
                /* 未知的 epoll 事件 */
                log_printf(LOG_DEBUG, DBG_EPOLL_UNKNOWN, ev->events, ev->data.fd);
            }
            return true; /* 事件已处理 */
        } /* if (ev->data.fd == interface->fd) */
    }
    return false; /* 不是 CAN 接口的事件 */
}
