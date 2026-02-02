/* CANopenNode 应用程序接口
 * 本文件定义了 CANopenNode 在 Linux 平台上的应用程序接口，提供类似 Arduino 的编程接口，
 * 并扩展了 CANopen 功能和实时线程支持。开发者可以通过实现这些回调函数来定制应用程序行为。
 */
/**
 * Application interface for CANopenNode.
 *
 * @file        CO_application.h
 * @ingroup     CO_applicationLinux
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
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

#ifndef CO_APPLICATION_H
#define CO_APPLICATION_H

#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_applicationLinux Application interface to CANopenNode Application interface, similar to Arduino,
 * extended to CANopen and additional, realtime thread.
 *
 * @ingroup CO_socketCAN
 * @{
 */

/* 程序启动时调用的函数
 * 函数功能：在程序启动时调用一次，发生在对象字典初始化之后、CANopen 初始化之前
 * 参数说明：
 *   - bitRate: 已存储的 CAN 总线波特率，可以在此函数中修改覆盖
 *   - nodeId: 已存储的 CANopen 节点 ID，可以在此函数中修改覆盖
 *   - errInfo: 输出错误信息，指示出错的对象字典条目索引
 * 返回值说明：成功返回 CO_ERROR_NO，否则返回相应的错误代码
 */
/**
 * Function is called once on the program startup, after Object dictionary initialization and before CANopen
 * initialization.
 *
 * @param [in,out] bitRate Stored CAN bit rate, can be overridden.
 * @param [in,out] nodeId Stored CANopen NodeId, can be overridden.
 * @param [out] errInfo Variable may indicate error information - index of erroneous OD entry.
 *
 * @return @ref CO_ReturnError_t CO_ERROR_NO in case of success.
 */
CO_ReturnError_t app_programStart(uint16_t* bitRate, uint8_t* nodeId, uint32_t* errInfo);

/* CANopen 通信复位后调用的函数
 * 函数功能：在 CANopen 通信复位后调用，用于执行应用层的复位相关操作
 * 参数说明：
 *   - co: CANopen 对象指针，包含所有 CANopen 协议栈的状态和配置
 * 返回值说明：无返回值
 */
/**
 * Function is called after CANopen communication reset.
 *
 * @param co CANopen object.
 */
void app_communicationReset(CO_t* co);

/* 程序结束前调用的函数
 * 函数功能：在程序即将结束时调用，用于执行清理和资源释放操作
 * 参数说明：无参数
 * 返回值说明：无返回值
 */
/**
 * Function is called just before program ends.
 */
void app_programEnd();

/* 主线程中周期性调用的函数（异步处理）
 * 函数功能：从 main() 函数中周期性调用，用于执行较慢的非实时代码（所有代码必须是非阻塞的）
 * 注意事项：需要注意与实时线程中运行的 app_programRt() 函数之间的竞争条件。如果访问的
 *         对象字典变量也可映射到 PDO，则必须使用 CO_LOCK_OD() 和 CO_UNLOCK_OD() 宏
 *         进行临界区保护
 * 参数说明：
 *   - co: CANopen 对象指针
 *   - timer1usDiff: 自上次调用以来经过的时间（以微秒为单位）
 * 返回值说明：无返回值
 */
/**
 * Function is called cyclically from main().
 *
 * Place for the slower code (all must be non-blocking).
 *
 * @warning
 * Mind race conditions between this functions and app_programRt(), which run from the realtime thread. If accessing
 * Object dictionary variable which is also mappable to PDO, it is necessary to use CO_LOCK_OD() and CO_UNLOCK_OD()
 * macros from @ref CO_critical_sections.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programAsync(CO_t* co, uint32_t timer1usDiff);

/* 实时线程中周期性调用的函数
 * 函数功能：从实时线程中以恒定时间间隔周期性调用，用于执行对时间要求严格的实时代码
 * 注意事项：此函数内的代码必须快速执行完成，需要注意与 app_programAsync 函数之间的竞争条件
 * 参数说明：
 *   - co: CANopen 对象指针
 *   - timer1usDiff: 自上次调用以来经过的时间（以微秒为单位）
 * 返回值说明：无返回值
 */
/**
 * Function is called cyclically from realtime thread at constant intervals.
 *
 * Code inside this function must be executed fast. Take care on race conditions with app_programAsync.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programRt(CO_t* co, uint32_t timer1usDiff);

/** @} */ /* CO_applicationLinux */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_APPLICATION_H */
