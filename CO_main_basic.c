/* CANopen Linux主程序文件 - 基于CANopenNode的Linux实现 */
/*
 * 文件功能说明:
 * 这是CANopen协议栈在Linux系统上的主程序实现文件。
 * 
 * 主要功能包括:
 * - 初始化CANopen协议栈和CAN总线通信
 * - 创建并管理主线程和实时线程(可选)
 * - 处理CANopen对象字典(OD)和SDO/PDO通信
 * - 提供数据持久化存储功能
 * - 支持LSS(层设置服务)进行节点ID和波特率配置
 * - 提供ASCII网关接口用于远程管理(可选)
 * - 处理NMT状态管理和心跳监控
 * 
 * 程序架构:
 * - 单线程模式: 所有任务在主线程中按顺序处理
 * - 多线程模式: 实时线程(高优先级)处理CAN接收和定时器
 *               主线程处理SDO、网关等非实时任务
 */
/*
 * CANopen main program file for CANopen Node on Linux.
 *
 * @file        CO_main_basic.c
 * @author      Janez Paternoster
 * @copyright   2020 Janez Paternoster
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>
#include <syslog.h>
#include <time.h>
#include <sys/epoll.h>
#include <net/if.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include "CANopen.h"
#include "OD.h"
#include "CO_error.h"
#include "CO_epoll_interface.h"
#include "CO_storageLinux.h"

/* 包含可选的外部应用程序函数 */
/* Include optional external application functions */
#ifdef CO_USE_APPLICATION
#include "CO_application.h"
#endif

/* 添加变量跟踪功能，用于记录变量随时间的变化 */
/* Add trace functionality for recording variables over time */
#if (CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE
#include "CO_time_trace.h"
#endif

/* 主线程间隔和实时线程间隔(微秒) */
/* Interval of mainline and real-time thread in microseconds */
#ifndef MAIN_THREAD_INTERVAL_US
#define MAIN_THREAD_INTERVAL_US 100000  /* 主线程间隔: 100ms */
#endif
#ifndef TMR_THREAD_INTERVAL_US
#define TMR_THREAD_INTERVAL_US 1000     /* 实时线程间隔: 1ms */
#endif

/* CO_CANopenInit()的默认值配置 */
/* default values for CO_CANopenInit() */
#ifndef NMT_CONTROL
/* NMT控制标志: 启动到操作状态，错误时报告，启用通用和通信错误 */
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME 500               /* 首次心跳时间: 500ms */
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000       /* SDO服务器超时: 1000ms */
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500        /* SDO客户端超时: 500ms */
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK false             /* SDO客户端块传输: 禁用 */
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS NULL             /* OD状态位: 无 */
#endif
/* CANopen网关使能开关，用于CO_epoll_processMain() */
/* CANopen gateway enable switch for CO_epoll_processMain() */
#ifndef GATEWAY_ENABLE
#define GATEWAY_ENABLE true             /* 网关功能: 启用 */
#endif
/* 时间戳消息间隔(毫秒) */
/* Interval for time stamp message in milliseconds */
#ifndef TIME_STAMP_INTERVAL_MS
#define TIME_STAMP_INTERVAL_MS 10000    /* 时间戳间隔: 10秒 */
#endif

/* 应用程序特定数据存储对象定义 */
/* Definitions for application specific data storage objects */
#ifndef CO_STORAGE_APPLICATION
#define CO_STORAGE_APPLICATION
#endif
/* 自动数据存储间隔(微秒) */
/* Interval for automatic data storage in microseconds */
#ifndef CO_STORAGE_AUTO_INTERVAL
#define CO_STORAGE_AUTO_INTERVAL 60000000  /* 自动存储间隔: 60秒 */
#endif

/* 全局CANopen对象指针 */
/* CANopen object */
CO_t* CO = NULL;

/* 活动节点ID，在通信复位时从pendingNodeId复制 */
/* Active node-id, copied from pendingNodeId in the communication reset */
static uint8_t CO_activeNodeId = CO_LSS_NODE_ID_ASSIGNMENT;

/* 主线数据块结构，可保存到非易失性存储器 */
/* Data block for mainline data, which can be stored to non-volatile memory */
typedef struct {
    /* 待处理的CAN波特率，可通过参数或LSS从站设置 */
    /* Pending CAN bit rate, can be set by argument or LSS slave. */
    uint16_t pendingBitRate;
    /* 待处理的CANopen节点ID，可通过参数或LSS从站设置 */
    /* Pending CANopen NodeId, can be set by argument or LSS slave. */
    uint8_t pendingNodeId;
} mainlineStorage_t;

/* 主线数据存储全局变量 */
mainlineStorage_t mlStorage = {0};

#if (CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE
/* 时间对象，用于记录当前时间 */
static CO_time_t CO_time; /* Object for current time */
#endif

/* 辅助函数 - 信号处理、日志记录、回调函数等 ********************************************/
/* Helper functions ***********************************************************/
#ifndef CO_SINGLE_THREAD
/* 实时线程epoll接口对象 */
/* Realtime thread */
CO_epoll_t epRT;
/* 实时线程函数声明 */
static void* rt_thread(void* arg);
#endif

/* 程序结束标志 - 由信号处理器设置，用于优雅退出程序 */
/* Signal handler */
volatile sig_atomic_t CO_endProgram = 0;

/*
 * 函数功能: 信号处理函数，用于处理SIGINT(Ctrl+C)和SIGTERM信号
 * 执行步骤:
 *   步骤1: 忽略具体信号值
 *   步骤2: 设置全局标志CO_endProgram为1，通知主循环退出
 * 参数说明:
 *   sig - 收到的信号编号
 * 返回值说明: 无返回值
 */
static void
sigHandler(int sig) {
    (void)sig;
    CO_endProgram = 1;
}

/*
 * 函数功能: 日志打印函数，将日志同时输出到系统日志和CANopen网关
 * 执行步骤:
 *   步骤1: 使用vsyslog将日志写入系统日志
 *   步骤2: 如果启用网关ASCII日志功能，格式化日志并发送到网关
 *   步骤3: 在网关日志中添加时间戳前缀(年-月-日 时:分:秒格式)
 * 参数说明:
 *   priority - 日志优先级(LOG_INFO, LOG_CRIT等)
 *   format - 格式化字符串
 *   ... - 可变参数列表
 * 返回值说明: 无返回值
 */
/* Message logging function */
void
log_printf(int priority, const char* format, ...) {
    va_list ap;

    /* 步骤1: 输出到系统日志 */
    va_start(ap, format);
    vsyslog(priority, format, ap);
    va_end(ap);

#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII_LOG
    /* 步骤2: 如果CANopen网关存在，同时输出到网关日志 */
    if (CO != NULL) {
        char buf[200];
        time_t timer;
        struct tm* tm_info;
        size_t len;

        /* 步骤3: 添加时间戳前缀 */
        timer = time(NULL);
        tm_info = localtime(&timer);
        len = strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S: ", tm_info);

        /* 格式化日志内容并输出到网关 */
        va_start(ap, format);
        vsnprintf(buf + len, sizeof(buf) - len - 2, format, ap);
        va_end(ap);
        CO_GTWA_log_print(CO->gtwa, buf);
    }
#endif
}

#if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
/*
 * 函数功能: 紧急消息接收回调函数，处理从其他节点接收到的紧急报文
 * 执行步骤:
 *   步骤1: 从标识符中提取发送节点ID(标识符低7位)
 *   步骤2: 记录紧急消息详细信息到日志
 * 参数说明:
 *   ident - CAN标识符，包含发送节点ID
 *   errorCode - 错误代码
 *   errorRegister - 错误寄存器值
 *   errorBit - 错误位
 *   infoCode - 附加信息代码
 * 返回值说明: 无返回值
 */
/* callback for emergency messages */
static void
EmergencyRxCallback(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit,
                    const uint32_t infoCode) {
    /* 步骤1: 提取发送节点ID */
    int16_t nodeIdRx = ident ? (ident & 0x7F) : CO_activeNodeId;

    /* 步骤2: 记录紧急消息 */
    log_printf(LOG_NOTICE, DBG_EMERGENCY_RX, nodeIdRx, errorCode, errorRegister, errorBit, infoCode);
}
#endif

#if ((CO_CONFIG_NMT)&CO_CONFIG_NMT_CALLBACK_CHANGE) || ((CO_CONFIG_HB_CONS)&CO_CONFIG_HB_CONS_CALLBACK_CHANGE)
/*
 * 函数功能: 将NMT状态枚举值转换为可读字符串
 * 参数说明:
 *   state - NMT内部状态枚举值
 * 返回值说明: NMT状态对应的字符串描述
 */
/* return string description of NMT state. */
static char*
NmtState2Str(CO_NMT_internalState_t state) {
    /* 根据NMT状态返回对应描述字符串 */
    switch (state) {
        case CO_NMT_INITIALIZING: return "initializing";      /* 初始化中 */
        case CO_NMT_PRE_OPERATIONAL: return "pre-operational"; /* 预操作状态 */
        case CO_NMT_OPERATIONAL: return "operational";         /* 操作状态 */
        case CO_NMT_STOPPED: return "stopped";                 /* 停止状态 */
        default: return "unknown";                             /* 未知状态 */
    }
}
#endif

#if (CO_CONFIG_NMT) & CO_CONFIG_NMT_CALLBACK_CHANGE
/*
 * 函数功能: NMT状态改变回调函数，当本节点NMT状态改变时被调用
 * 参数说明:
 *   state - 新的NMT状态
 * 返回值说明: 无返回值
 */
/* callback for NMT change messages */
static void
NmtChangedCallback(CO_NMT_internalState_t state) {
    /* 记录NMT状态变化到日志 */
    log_printf(LOG_NOTICE, DBG_NMT_CHANGE, NmtState2Str(state), state);
}
#endif

#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_CALLBACK_CHANGE
/*
 * 函数功能: 心跳消费者回调函数，监控远程节点的NMT状态变化
 * 参数说明:
 *   nodeId - 远程节点ID
 *   idx - 心跳索引
 *   state - 远程节点的新NMT状态
 *   object - 用户对象指针(未使用)
 * 返回值说明: 无返回值
 */
/* callback for monitoring Heartbeat remote NMT state change */
static void
HeartbeatNmtChangedCallback(uint8_t nodeId, uint8_t idx, CO_NMT_internalState_t state, void* object) {
    (void)object;
    /* 记录远程节点心跳状态变化 */
    log_printf(LOG_NOTICE, DBG_HB_CONS_NMT_CHANGE, nodeId, idx, NmtState2Str(state), state);
}
#endif

/*
 * 函数功能: LSS配置存储回调函数，保存LSS从站配置的节点ID和波特率
 * 执行步骤:
 *   步骤1: 将传入的object指针转换为mainlineStorage_t类型
 *   步骤2: 保存新配置的节点ID到存储结构
 *   步骤3: 保存新配置的CAN波特率到存储结构
 *   步骤4: 返回true表示存储成功
 * 参数说明:
 *   object - 指向mainlineStorage_t结构的指针
 *   id - LSS配置的新节点ID
 *   bitRate - LSS配置的新CAN波特率
 * 返回值说明: true表示存储成功
 */
/* callback for storing node id and bitrate */
static bool_t
LSScfgStoreCallback(void* object, uint8_t id, uint16_t bitRate) {
    /* 步骤1: 获取主线数据存储指针 */
    mainlineStorage_t* mainlineStorage = object;
    /* 步骤2: 保存待处理的节点ID */
    mainlineStorage->pendingNodeId = id;
    /* 步骤3: 保存待处理的波特率 */
    mainlineStorage->pendingBitRate = bitRate;
    /* 步骤4: 返回存储成功 */
    return true;
}

/*
 * 函数功能: 打印程序使用说明和命令行参数帮助信息
 * 参数说明:
 *   progName - 程序名称
 * 返回值说明: 无返回值
 */
/* Print usage */
static void
printUsage(char* progName) {
    /* 打印基本用法 */
    printf("Usage: %s [options] <CAN device name>\n", progName);
    printf("\n"
           "Options:\n"
           "  -i <Node ID>        CANopen Node-id (1..127) or 0xFF (LSS unconfigured).\n");
#ifndef CO_SINGLE_THREAD
    /* 多线程模式: 显示实时线程优先级选项 */
    printf("  -p <RT priority>    Real-time priority of RT thread (1 .. 99). If not set or\n"
           "                      set to -1, then normal scheduler is used for RT thread.\n");
#endif
    /* 打印重启选项 */
    printf("  -r                  Enable reboot on CANopen NMT reset_node command. \n");
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    /* 启用存储功能: 显示存储路径选项 */
    printf("  -s <storage path>   Path and filename prefix for data storage files.\n"
           "                      By default files are stored in current dictionary.\n");
#endif
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
    /* 启用网关功能: 显示命令接口选项 */
    printf("  -c <interface>      Enable command interface for master functionality.\n"
           "                      One of three types of interfaces can be specified as:\n"
           "                   1. \"stdio\" - Standard IO of a program (terminal).\n"
           "                   2. \"local-<file path>\" - Local socket interface on file\n"
           "                      path, for example \"local-/tmp/CO_command_socket\".\n"
           "                   3. \"tcp-<port>\" - Tcp socket interface on specified \n"
           "                      port, for example \"tcp-60000\".\n"
           "                      Note that this option may affect security of the CAN.\n"
           "  -T <timeout_time>   If -c is specified as local or tcp socket, then this\n"
           "                      parameter specifies socket timeout time in milliseconds.\n"
           "                      Default is 0 - no timeout on established connection.\n");
#endif
    /* 打印项目链接 */
    printf("\n"
           "See also: https://github.com/CANopenNode/CANopenNode\n"
           "\n");
}

/*******************************************************************************
 * 主线程 - CANopen应用程序的主函数
 * 
 * 函数功能: CANopen主程序入口，负责初始化、配置和运行CANopen协议栈
 * 
 * 主要执行流程:
 *   第一阶段: 程序初始化和参数解析(步骤1-14)
 *   第二阶段: CANopen对象创建和配置(步骤15-21)
 *   第三阶段: 通信复位循环 - 初始化CANopen通信(步骤22-30)
 *   第四阶段: 正常运行循环 - 处理CANopen通信和应用逻辑(步骤31-35)
 *   第五阶段: 程序退出清理(步骤36-40)
 * 
 * 参数说明:
 *   argc - 命令行参数个数
 *   argv - 命令行参数数组
 * 
 * 返回值说明: EXIT_SUCCESS(0)表示成功，EXIT_FAILURE(1)表示失败
 ******************************************************************************/
int
main(int argc, char* argv[]) {
    /* ===== 变量声明区域 ===== */
    int programExit = EXIT_SUCCESS; /* 程序退出码 */
    CO_epoll_t epMain;              /* 主线程epoll接口对象 */
#ifndef CO_SINGLE_THREAD
    pthread_t rt_thread_id;         /* 实时线程ID */
    int rtPriority = -1;            /* 实时线程优先级(-1表示使用普通调度器) */
#endif
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT; /* NMT复位命令 */
    CO_ReturnError_t err;                     /* 错误返回值 */
    CO_CANptrSocketCan_t CANptr = {0};       /* SocketCAN指针结构 */
    int opt;                                  /* getopt返回的选项字符 */
    bool_t firstRun = true;                  /* 首次运行标志 */

    /* 可通过命令行参数配置的变量 */
    char* CANdevice = NULL;      /* CAN device, configurable by arguments. */
    int16_t nodeIdFromArgs = -1; /* May be set by arguments */
    bool_t rebootEnable = false; /* Configurable by arguments */

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    /* 数据存储相关变量 */
    CO_storage_t storage;                    /* 存储对象 */
    CO_storage_entry_t storageEntries[] = {  /* 存储条目数组 */
        /* 条目1: OD通信参数持久化存储 */
        {.addr = &OD_PERSIST_COMM,
         .len = sizeof(OD_PERSIST_COMM),
         .subIndexOD = 2,
         .attr = CO_storage_cmd | CO_storage_restore,
         .filename = {'o', 'd', '_', 'c', 'o', 'm', 'm', '.', 'p', 'e', 'r', 's', 'i', 's', 't', '\0'}},
        /* 条目2: 主线数据(节点ID和波特率)持久化存储 */
        {.addr = &mlStorage,
         .len = sizeof(mlStorage),
         .subIndexOD = 4,
         .attr = CO_storage_cmd | CO_storage_auto | CO_storage_restore,
         .filename = {'m', 'a', 'i', 'n', 'l', 'i', 'n', 'e', '.', 'p', 'e', 'r', 's', 'i', 's', 't', '\0'}},
        /* 应用自定义存储条目(如果有) */
        CO_STORAGE_APPLICATION};
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]); /* 存储条目数量 */
    uint32_t storageInitError = 0;          /* 存储初始化错误 */
    uint32_t storageErrorPrev = 0;          /* 前一次存储错误 */
    uint32_t storageIntervalTimer = 0;      /* 自动存储间隔计时器 */
#endif

#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
    /* CANopen网关相关变量 */
    CO_epoll_gtw_t epGtw;                              /* 网关epoll接口对象 */
    int32_t commandInterface = CO_COMMAND_IF_DISABLED; /* 命令接口类型(CO_commandInterface_t枚举值) */
    char* localSocketPath = NULL;                      /* 本地socket路径(当commandInterface为LOCAL_SOCKET时使用) */
    uint32_t socketTimeout_ms = 0;                     /* socket超时时间(毫秒) */
#else
#define commandInterface 0
#define localSocketPath  NULL
#endif

    /* ===== 第一阶段: 程序初始化和参数解析 ===== */
    
    /* 步骤1: 配置系统日志 */
    setlogmask(LOG_UPTO(LOG_DEBUG));                  /* LOG_DEBUG - log all messages */
    openlog(argv[0], LOG_PID | LOG_PERROR, LOG_USER); /* print also to standard error */

    /* 步骤2: 获取并解析程序选项 */
    /* Get program options */
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        /* 如果没有参数或请求帮助，显示使用说明并退出 */
        printUsage(argv[0]);
        exit(EXIT_SUCCESS);
    }
    /* 循环解析所有命令行选项 */
    while ((opt = getopt(argc, argv, "i:p:rc:T:s:")) != -1) {
        switch (opt) {
            case 'i': {
                /* 选项i: 设置CANopen节点ID (1-127或0xFF表示未配置) */
                long int nodeIdLong = strtol(optarg, NULL, 0);
                nodeIdFromArgs = (nodeIdLong < 0 || nodeIdLong > 0xFF) ? 0 : (uint8_t)strtol(optarg, NULL, 0);
                break;
            }
#ifndef CO_SINGLE_THREAD
            case 'p': 
                /* 选项p: 设置实时线程优先级 (1-99) */
                rtPriority = strtol(optarg, NULL, 0); 
                break;
#endif
            case 'r': 
                /* 选项r: 启用NMT复位节点命令时重启系统 */
                rebootEnable = true; 
                break;
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
            case 'c': {
                /* 选项c: 配置命令接口类型(stdio/local socket/tcp socket) */
                const char* comm_stdio = "stdio";
                const char* comm_local = "local-";
                const char* comm_tcp = "tcp-";
                if (strcmp(optarg, comm_stdio) == 0) {
                    /* 使用标准输入输出作为命令接口 */
                    commandInterface = CO_COMMAND_IF_STDIO;
                } else if (strncmp(optarg, comm_local, strlen(comm_local)) == 0) {
                    /* 使用本地socket作为命令接口 */
                    commandInterface = CO_COMMAND_IF_LOCAL_SOCKET;
                    localSocketPath = &optarg[6];
                } else if (strncmp(optarg, comm_tcp, strlen(comm_tcp)) == 0) {
                    /* 使用TCP socket作为命令接口 */
                    const char* portStr = &optarg[4];
                    uint16_t port;
                    int nMatch = sscanf(portStr, "%hu", &port);
                    if (nMatch != 1) {
                        log_printf(LOG_CRIT, DBG_NOT_TCP_PORT, portStr);
                        exit(EXIT_FAILURE);
                    }
                    commandInterface = port;
                } else {
                    /* 未知的命令接口类型 */
                    log_printf(LOG_CRIT, DBG_ARGUMENT_UNKNOWN, "-c", optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            }
            case 'T': 
                /* 选项T: 设置socket超时时间(毫秒) */
                socketTimeout_ms = strtoul(optarg, NULL, 0); 
                break;
#endif
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            case 's': {
                /* 选项s: 为存储文件添加路径前缀 */
                /* add prefix to each storageEntries[i].filename */
                for (uint8_t i = 0; i < storageEntriesCount; i++) {
                    char* filePrefix = optarg;
                    size_t filePrefixLen = strlen(filePrefix);
                    char* file = storageEntries[i].filename;
                    size_t fileLen = strlen(file);
                    if (fileLen + filePrefixLen < CO_STORAGE_PATH_MAX) {
                        /* 在文件名前添加路径前缀 */
                        memmove(&file[filePrefixLen], &file[0], fileLen + 1);
                        memcpy(&file[0], &filePrefix[0], filePrefixLen);
                    }
                }
                break;
            }
#endif
            default: 
                /* 未知选项，打印使用说明并退出 */
                printUsage(argv[0]); 
                exit(EXIT_FAILURE);
        }
    }

    /* 步骤3: 获取CAN设备名称并转换为接口索引 */
    if (optind < argc) {
        CANdevice = argv[optind];
        CANptr.can_ifindex = if_nametoindex(CANdevice);
    }

    /* 步骤4: 验证节点ID的有效性 */
    /* Valid NodeId is 1..127 or 0xFF(unconfigured) in case of LSSslaveEnabled */
    if ((nodeIdFromArgs == 0 || nodeIdFromArgs > 127)
        && (!CO_isLSSslaveEnabled(CO) || nodeIdFromArgs != CO_LSS_NODE_ID_ASSIGNMENT)) {
        log_printf(LOG_CRIT, DBG_WRONG_NODE_ID, nodeIdFromArgs);
        printUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

#ifndef CO_SINGLE_THREAD
    /* 步骤5: 验证实时线程优先级的有效性 */
    if (rtPriority != -1
        && (rtPriority < sched_get_priority_min(SCHED_FIFO) || rtPriority > sched_get_priority_max(SCHED_FIFO))) {
        log_printf(LOG_CRIT, DBG_WRONG_PRIORITY, rtPriority);
        printUsage(argv[0]);
        exit(EXIT_FAILURE);
    }
#endif

    /* 步骤6: 验证CAN设备是否存在 */
    if (CANptr.can_ifindex == 0) {
        log_printf(LOG_CRIT, DBG_NO_CAN_DEVICE, CANdevice);
        exit(EXIT_FAILURE);
    }

    /* 步骤7: 记录程序启动日志 */
    log_printf(LOG_INFO, DBG_CAN_OPEN_INFO, mlStorage.pendingNodeId, "starting");

    /* ===== 第二阶段: CANopen对象创建和配置 ===== */
    
    /* 步骤8: 为CANopen对象分配内存 */
    /* Allocate memory for CANopen objects */
    uint32_t heapMemoryUsed = 0;
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* 使用多对象字典模式配置CANopen对象数量 */
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
#if (CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE
    co_config.CNT_LEDS = 1;    /* LED对象数量 */
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE
    co_config.CNT_LSS_SLV = 1; /* LSS从站对象数量 */
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_MASTER
    co_config.CNT_LSS_MST = 1; /* LSS主站对象数量 */
#endif
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
    co_config.CNT_GTWA = 1;    /* ASCII网关对象数量 */
#endif
#if (CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE
    co_config.CNT_TRACE = 1;   /* 跟踪对象数量 */
#endif
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */
    /* 创建CANopen主对象 */
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf(LOG_CRIT, DBG_GENERAL, "CO_new(), heapMemoryUsed=", heapMemoryUsed);
        exit(EXIT_FAILURE);
    }

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    /* 步骤9: 初始化数据存储功能 */
    err = CO_storageLinux_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        /* 存储初始化失败(数据损坏可以容忍) */
        char* filename = storageInitError < storageEntriesCount ? storageEntries[storageInitError].filename : "???";
        log_printf(LOG_CRIT, DBG_STORAGE, filename);
        exit(EXIT_FAILURE);
    }
#endif
#ifdef CO_USE_APPLICATION
    /* 步骤10: 执行可选的外部应用程序启动代码 */
    /* Execute optional external application code */
    uint32_t errInfo_app_programStart = 0;
    err = app_programStart(&mlStorage.pendingBitRate, &mlStorage.pendingNodeId, &errInfo_app_programStart);
    if (err != CO_ERROR_NO) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf(LOG_CRIT, DBG_OD_ENTRY, errInfo_app_programStart);
        } else {
            log_printf(LOG_CRIT, DBG_CAN_OPEN, "app_programStart()", err);
        }
        exit(EXIT_FAILURE);
    }
#endif

    /* 步骤11: 如果命令行指定了节点ID，覆盖存储的节点ID */
    /* Overwrite node-id, if specified by program arguments */
    if (nodeIdFromArgs > 0) {
        mlStorage.pendingNodeId = (uint8_t)nodeIdFromArgs;
    }
    /* 步骤12: 验证并修正存储的节点ID */
    /* verify stored values */
    if (mlStorage.pendingNodeId < 1 || mlStorage.pendingNodeId > 127) {
        mlStorage.pendingNodeId = CO_LSS_NODE_ID_ASSIGNMENT;
    }

    /* 步骤13: 捕获SIGINT(Ctrl+C)和SIGTERM信号以实现优雅退出 */
    /* Catch signals SIGINT and SIGTERM */
    if (signal(SIGINT, sigHandler) == SIG_ERR) {
        log_printf(LOG_CRIT, DBG_ERRNO, "signal(SIGINT, sigHandler)");
        exit(EXIT_FAILURE);
    }
    if (signal(SIGTERM, sigHandler) == SIG_ERR) {
        log_printf(LOG_CRIT, DBG_ERRNO, "signal(SIGTERM, sigHandler)");
        exit(EXIT_FAILURE);
    }

    /* 步骤14: 获取当前时间用于CANopen TIME对象 */
    /* get current time for CO_TIME_set(), since January 1, 1984, UTC. */
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
        log_printf(LOG_CRIT, DBG_GENERAL, "clock_gettime(main)", 0);
        exit(EXIT_FAILURE);
    }
    /* 计算自CANopen纪元(1984年1月1日)以来的天数 */
    uint16_t time_days = (uint16_t)(ts.tv_sec / (24 * 60 * 60));
    time_days -= 5113; /* difference between Unix epoch and CANopen Epoch */
    /* 计算当天的毫秒数 */
    uint32_t time_ms = (uint32_t)(ts.tv_sec % (24 * 60 * 60)) * 1000;
    time_ms += ts.tv_nsec / 1000000;

    /* 步骤15: 创建epoll接口对象 */
    /* Create epoll functions */
    /* 创建主线程epoll接口(间隔100ms) */
    err = CO_epoll_create(&epMain, MAIN_THREAD_INTERVAL_US);
    if (err != CO_ERROR_NO) {
        log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_create(main), err=", err);
        exit(EXIT_FAILURE);
    }
#ifndef CO_SINGLE_THREAD
    /* 创建实时线程epoll接口(间隔1ms) */
    err = CO_epoll_create(&epRT, TMR_THREAD_INTERVAL_US);
    if (err != CO_ERROR_NO) {
        log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_create(RT), err=", err);
        exit(EXIT_FAILURE);
    }
    CANptr.epoll_fd = epRT.epoll_fd;
#else
    /* 单线程模式: CAN使用主线程的epoll */
    CANptr.epoll_fd = epMain.epoll_fd;
#endif
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
    /* 步骤16: 创建网关epoll接口 */
    err = CO_epoll_createGtw(&epGtw, epMain.epoll_fd, commandInterface, socketTimeout_ms, localSocketPath);
    if (err != CO_ERROR_NO) {
        log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_createGtw(), err=", err);
        exit(EXIT_FAILURE);
    }
#endif

    /* ===== 第三阶段: 通信复位循环 - 初始化CANopen通信 ===== */
    /* 外层循环: 处理CANopen通信复位，直到收到退出命令 */
    while (reset != CO_RESET_APP && reset != CO_RESET_QUIT && CO_endProgram == 0) {
        /* 步骤17: CANopen通信复位 - 初始化CANopen对象 ********************/
        /* CANopen communication reset - initialize CANopen objects *******************/
        uint32_t errInfo;

        /* 如果不是首次运行，等待实时线程停止CAN通信 */
        /* Wait rt_thread. */
        if (!firstRun) {
            CO_LOCK_OD(CO->CANmodule);
            CO->CANmodule->CANnormal = false;
            CO_UNLOCK_OD(CO->CANmodule);
        }

        /* 步骤18: 进入CAN配置模式并禁用CAN模块 */
        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode((void*)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* 步骤19: 初始化CAN接口 */
        /* initialize CANopen */
        err = CO_CANinit(CO, (void*)&CANptr, 0 /* bit rate not used */);
        if (err != CO_ERROR_NO) {
            log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANinit()", err);
            programExit = EXIT_FAILURE;
            CO_endProgram = 1;
            continue;
        }

        /* 步骤20: 初始化LSS(层设置服务)功能 */
        /* 从对象字典中读取LSS地址信息(厂商ID、产品代码、版本号、序列号) */
        CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                    .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                    .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                    .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
        err = CO_LSSinit(CO, &lssAddress, &mlStorage.pendingNodeId, &mlStorage.pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_LSSinit()", err);
            programExit = EXIT_FAILURE;
            CO_endProgram = 1;
            continue;
        }

        /* 步骤21: 设置活动节点ID */
        CO_activeNodeId = mlStorage.pendingNodeId;
        errInfo = 0;

        /* 步骤22: 初始化CANopen协议栈(不包含PDO) */
        err = CO_CANopenInit(CO,                   /* CANopen object - CANopen对象 */
                             NULL,                 /* alternate NMT - 备用NMT(未使用) */
                             NULL,                 /* alternate em - 备用紧急对象(未使用) */
                             OD,                   /* Object dictionary - 对象字典 */
                             OD_STATUS_BITS,       /* Optional OD_statusBits - 可选状态位 */
                             NMT_CONTROL,          /* CO_NMT_control_t - NMT控制标志 */
                             FIRST_HB_TIME,        /* firstHBTime_ms - 首次心跳时间(毫秒) */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms - SDO服务器超时 */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms - SDO客户端超时 */
                             SDO_CLI_BLOCK,        /* SDOclientBlockTransfer - SDO块传输使能 */
                             CO_activeNodeId, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            /* 初始化失败(LSS未配置节点ID错误除外) */
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf(LOG_CRIT, DBG_OD_ENTRY, errInfo);
            } else {
                log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANopenInit()", err);
            }
            programExit = EXIT_FAILURE;
            CO_endProgram = 1;
            continue;
        }

        /* 步骤23: 初始化主线程处理函数和回调函数 */
        /* initialize part of threadMain and callbacks */
        CO_epoll_initCANopenMain(&epMain, CO);
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
        /* 初始化网关 */
        CO_epoll_initCANopenGtw(&epGtw, CO);
#endif
        /* 初始化LSS配置存储回调 */
        CO_LSSslave_initCfgStoreCall(CO->LSSslave, &mlStorage, LSScfgStoreCallback);
        
        if (!CO->nodeIdUnconfigured) {
            /* 节点ID已配置的情况下初始化各种回调和错误报告 */
            if (errInfo != 0) {
                /* 报告对象字典不一致错误 */
                CO_errorReport(CO->em, CO_EM_INCONSISTENT_OBJECT_DICT, CO_EMC_DATA_SET, errInfo);
            }
#if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
            /* 注册紧急消息接收回调 */
            CO_EM_initCallbackRx(CO->em, EmergencyRxCallback);
#endif
#if (CO_CONFIG_NMT) & CO_CONFIG_NMT_CALLBACK_CHANGE
            /* 注册NMT状态改变回调 */
            CO_NMT_initCallbackChanged(CO->NMT, NmtChangedCallback);
#endif
#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_CALLBACK_CHANGE
            /* 注册心跳消费者状态改变回调 */
            CO_HBconsumer_initCallbackNmtChanged(CO->HBcons, 0, NULL, HeartbeatNmtChangedCallback);
#endif
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if (storageInitError != 0) {
                /* 报告存储初始化错误 */
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
            }
#endif
#ifdef CO_USE_APPLICATION
            if (errInfo_app_programStart != 0) {
                /* 报告应用程序启动错误 */
                CO_errorReport(CO->em, CO_EM_INCONSISTENT_OBJECT_DICT, CO_EMC_DATA_SET, errInfo_app_programStart);
            }
#endif

#if (CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE
            /* 初始化时间跟踪对象 */
            /* Initialize time */
            CO_time_init(&CO_time, CO->SDO[0], &OD_time.epochTimeBaseMs, &OD_time.epochTimeOffsetMs, 0x2130);
#endif
            log_printf(LOG_INFO, DBG_CAN_OPEN_INFO, CO_activeNodeId, "communication reset");
        } else {
            /* 节点ID未配置(LSS模式) */
            log_printf(LOG_INFO, DBG_CAN_OPEN_INFO, CO_activeNodeId, "node-id not initialized");
        }

        /* 步骤24: 首次运行初始化(只执行一次) */
        /* First time only initialization. */
        if (firstRun) {
            firstRun = false;
            /* 设置CANopen TIME对象的初始值 */
            CO_TIME_set(CO->TIME, time_ms, time_days, TIME_STAMP_INTERVAL_MS);
#ifndef CO_SINGLE_THREAD
            /* 创建实时线程并设置优先级 */
            /* Create rt_thread and set priority */
            if (pthread_create(&rt_thread_id, NULL, rt_thread, NULL) != 0) {
                log_printf(LOG_CRIT, DBG_ERRNO, "pthread_create(rt_thread)");
                programExit = EXIT_FAILURE;
                CO_endProgram = 1;
                continue;
            }
            if (rtPriority > 0) {
                /* 设置实时线程为FIFO调度策略 */
                struct sched_param param;

                param.sched_priority = rtPriority;
                if (pthread_setschedparam(rt_thread_id, SCHED_FIFO, &param) != 0) {
                    log_printf(LOG_CRIT, DBG_ERRNO, "pthread_setschedparam()");
                    programExit = EXIT_FAILURE;
                    CO_endProgram = 1;
                    continue;
                }
            }
#endif
        } /* if(firstRun) */

#ifdef CO_USE_APPLICATION
        /* 步骤25: 执行应用程序通信复位代码 */
        /* Execute optional external application code */
        app_communicationReset(CO);
#endif

        /* 步骤26: 初始化PDO(过程数据对象) */
        errInfo = 0;
        err = CO_CANopenInitPDO(CO,     /* CANopen object - CANopen对象 */
                                CO->em, /* emergency object - 紧急对象 */
                                OD,     /* Object dictionary - 对象字典 */
                                CO_activeNodeId, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            /* PDO初始化失败 */
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf(LOG_CRIT, DBG_OD_ENTRY, errInfo);
            } else {
                log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANopenInitPDO()", err);
            }
            programExit = EXIT_FAILURE;
            CO_endProgram = 1;
            continue;
        }

        /* 步骤27: 启动CAN通信，进入正常模式 */
        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        log_printf(LOG_INFO, DBG_CAN_OPEN_INFO, CO_activeNodeId, "running ...");

        /* ===== 第四阶段: 正常运行循环 - 处理CANopen通信和应用逻辑 ===== */
        /* 内层循环: 正常程序执行，直到收到复位命令或退出信号 */
        while (reset == CO_RESET_NOT && CO_endProgram == 0) {
            /* 步骤28: 主线程循环处理 ******************************************/
            /* loop for normal program execution ******************************************/
            
            /* 等待epoll事件(定时器或文件描述符事件) */
            CO_epoll_wait(&epMain);
#ifdef CO_SINGLE_THREAD
            /* 单线程模式: 在主线程中处理实时任务 */
            CO_epoll_processRT(&epMain, CO, false);
#endif
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
            /* 处理网关通信(命令接口) */
            CO_epoll_processGtw(&epGtw, CO, &epMain);
#endif
            /* 处理主线程CANopen任务(SDO、心跳等) */
            CO_epoll_processMain(&epMain, CO, GATEWAY_ENABLE, &reset);
            /* 处理最后的清理任务 */
            CO_epoll_processLast(&epMain);

#ifdef CO_USE_APPLICATION
            /* 执行应用程序异步代码 */
            /* Execute optional external application code */
            app_programAsync(CO, epMain.timeDifference_us);
#endif

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            /* 步骤29: 处理自动数据存储(按设定间隔) */
            /* don't save more often than interval */
            if (storageIntervalTimer < CO_STORAGE_AUTO_INTERVAL) {
                /* 累加时间，未到存储间隔 */
                storageIntervalTimer += epMain.timeDifference_us;
            } else {
                /* 到达存储间隔，执行自动存储 */
                uint32_t mask = CO_storageLinux_auto_process(&storage, false);
                if (mask != storageErrorPrev && !CO->nodeIdUnconfigured) {
                    if (mask != 0) {
                        /* 报告存储错误 */
                        CO_errorReport(CO->em, CO_EM_NON_VOLATILE_AUTO_SAVE, CO_EMC_HARDWARE, mask);
                    } else {
                        /* 清除之前的存储错误 */
                        CO_errorReset(CO->em, CO_EM_NON_VOLATILE_AUTO_SAVE, 0);
                    }
                }
                storageErrorPrev = mask;
                storageIntervalTimer = 0; /* 重置计时器 */
            }
#endif
        }
    } /* while(reset != CO_RESET_APP */ /* 通信复位循环结束 */

    /* ===== 第五阶段: 程序退出清理 ===== */
    /* program exit ***************************************************************/
    
    /* 步骤30: 设置退出标志并等待线程结束 */
    /* join threads */
    CO_endProgram = 1;
#ifndef CO_SINGLE_THREAD
    /* 等待实时线程退出 */
    if (pthread_join(rt_thread_id, NULL) != 0) {
        log_printf(LOG_CRIT, DBG_ERRNO, "pthread_join()");
        exit(EXIT_FAILURE);
    }
#endif
#ifdef CO_USE_APPLICATION
    /* 步骤31: 执行应用程序退出代码 */
    /* Execute optional external application code */
    app_programEnd();
#endif

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    /* 步骤32: 强制保存所有待存储数据 */
    CO_storageLinux_auto_process(&storage, true);
#endif

    /* 步骤33: 清理并释放所有对象 */
    /* delete objects from memory */
#ifndef CO_SINGLE_THREAD
    /* 关闭实时线程epoll接口 */
    CO_epoll_close(&epRT);
#endif
    /* 关闭主线程epoll接口 */
    CO_epoll_close(&epMain);
#if (CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII
    /* 关闭网关epoll接口 */
    CO_epoll_closeGtw(&epGtw);
#endif
    /* 进入CAN配置模式并删除CANopen对象 */
    CO_CANsetConfigurationMode((void*)&CANptr);
    CO_delete(CO);

    /* 步骤34: 记录程序结束日志 */
    log_printf(LOG_INFO, DBG_CAN_OPEN_INFO, CO_activeNodeId, "finished");

    /* 步骤35: 如果启用重启并收到应用复位命令，则重启系统 */
    /* Flush all buffers (and reboot) */
    if (rebootEnable && reset == CO_RESET_APP) {
        sync(); /* 同步所有文件系统缓冲区 */
        if (reboot(LINUX_REBOOT_CMD_RESTART) != 0) {
            log_printf(LOG_CRIT, DBG_ERRNO, "reboot()");
            exit(EXIT_FAILURE);
        }
    }

    /* 步骤36: 退出程序 */
    exit(programExit);
}

#ifndef CO_SINGLE_THREAD
/*******************************************************************************
 * 实时线程 - 用于CAN接收和定时器处理
 * 
 * 函数功能: 实时线程函数，以高优先级处理时间关键的CANopen任务
 * 
 * 执行步骤:
 *   步骤1: 等待epoll事件(CAN消息接收或定时器到期)
 *   步骤2: 处理实时任务(CAN接收、SYNC、PDO、定时器回调等)
 *   步骤3: 处理时间跟踪对象(如果启用)
 *   步骤4: 执行应用程序实时代码(如果有)
 *   步骤5: 执行清理任务
 *   步骤6: 循环执行直到收到退出信号
 * 
 * 参数说明:
 *   arg - 线程参数(未使用)
 * 
 * 返回值说明: NULL
 ******************************************************************************/
static void*
rt_thread(void* arg) {
    (void)arg;
    
    /* 无限循环，直到收到程序退出信号 */
    /* Endless loop */
    while (CO_endProgram == 0) {

        /* 步骤1: 等待epoll事件(1ms间隔或CAN消息到达) */
        CO_epoll_wait(&epRT);
        
        /* 步骤2: 处理实时CANopen任务 */
        /* 包括: CAN消息接收、SYNC处理、RPDO处理、TPDO触发、定时器回调 */
        CO_epoll_processRT(&epRT, CO, true);
        
        /* 步骤5: 执行清理任务 */
        CO_epoll_processLast(&epRT);

#if (CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE
        /* 步骤3: 处理变量跟踪对象(用于数据记录) */
        /* Monitor variables with trace objects */
        CO_time_process(&CO_time);
        for (i = 0; i < OD_traceEnable && i < co->CNT_TRACE; i++) {
            CO_trace_process(CO->trace[i], *CO_time.epochTimeOffsetMs);
        }
#endif

#ifdef CO_USE_APPLICATION
        /* 步骤4: 执行应用程序实时代码 */
        /* Execute optional external application code */
        app_programRt(CO, epRT.timeDifference_us);
#endif
    }

    return NULL;
}
#endif
