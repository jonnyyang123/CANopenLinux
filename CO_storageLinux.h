/* Linux 平台的 CANopen 数据存储对象
 * 本文件提供 Linux 平台的数据持久化存储功能，支持将 CANopen 对象字典数据保存到
 * 文件系统中，并在启动时恢复。支持自动存储、CRC 校验和存储/恢复默认参数等功能。
 */
/**
 * CANopen data storage object for Linux
 *
 * @file        CO_storageLinux.h
 * @ingroup     CO_storageLinux
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

#ifndef CO_STORAGE_LINUX_H
#define CO_STORAGE_LINUX_H

#include "storage/CO_storage.h"

#if ((CO_CONFIG_STORAGE)&CO_CONFIG_STORAGE_ENABLE) || defined CO_DOXYGEN

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_storageLinux Data storage with Linux
 * Data initialize, store and restore functions with Linux.
 *
 * @ingroup CO_socketCAN
 * @{
 * See also @ref CO_storage.
 */

/* 初始化数据存储对象（Linux 平台特定）
 * 函数功能：初始化数据存储对象，设置对象字典扩展（0x1010 和 0x1011），从文件读取数据，
 *         验证数据并写入到条目指定的地址。此函数内部调用 CO_storage_init()
 * 使用说明：应该在程序启动后、CO_CANopenInit() 之前由应用程序调用
 * 参数说明：
 *   - storage: 要初始化的存储对象，必须由应用程序定义并永久存在
 *   - CANmodule: CAN 设备，用于 CO_LOCK_OD() 宏
 *   - OD_1010_StoreParameters: 0x1010 "存储参数"对象字典条目，可选，可为 NULL
 *   - OD_1011_RestoreDefaultParam: 0x1011 "恢复默认参数"对象字典条目，可选，可为 NULL
 *   - entries: 存储条目数组指针，参见 CO_storage_init
 *   - entriesCount: 存储条目数量
 *   - storageInitError: [输出] 如果函数返回 CO_ERROR_DATA_CORRUPT，则此变量包含 subIndexOD
 *                      值的位掩码，表示数据未能正确初始化。如果是其他错误，则包含出错条目的索引
 * 返回值说明：
 *   - CO_ERROR_NO: 成功
 *   - CO_ERROR_DATA_CORRUPT: 数据无法初始化
 *   - CO_ERROR_ILLEGAL_ARGUMENT: 参数非法
 *   - CO_ERROR_OUT_OF_MEMORY: 内存不足
 */
/**
 * Initialize data storage object (Linux specific)
 *
 * This function should be called by application after the program startup, before @ref CO_CANopenInit(). This function
 * initializes storage object, OD extensions on objects 1010 and 1011, reads data from file, verifies them and writes
 * data to addresses specified inside entries. This function internally calls @ref CO_storage_init().
 *
 * @param storage This object will be initialized. It must be defined by application and must exist permanently.
 * @param CANmodule CAN device, used for @ref CO_LOCK_OD() macro.
 * @param OD_1010_StoreParameters OD entry for 0x1010 -"Store parameters". Entry is optional, may be NULL.
 * @param OD_1011_RestoreDefaultParam OD entry for 0x1011 -"Restore default parameters". Entry is optional, may be NULL.
 * @param entries Pointer to array of storage entries, see @ref CO_storage_init.
 * @param entriesCount Count of storage entries
 * @param [out] storageInitError If function returns CO_ERROR_DATA_CORRUPT, then this variable contains a bit mask from
 * subIndexOD values, where data was not properly initialized. If other error, then this variable contains index or
 * erroneous entry.
 *
 * @return CO_ERROR_NO, CO_ERROR_DATA_CORRUPT if data can not be initialized, CO_ERROR_ILLEGAL_ARGUMENT or
 * CO_ERROR_OUT_OF_MEMORY.
 */
CO_ReturnError_t CO_storageLinux_init(CO_storage_t* storage, CO_CANmodule_t* CANmodule,
                                      OD_entry_t* OD_1010_StoreParameters, OD_entry_t* OD_1011_RestoreDefaultParam,
                                      CO_storage_entry_t* entries, uint8_t entriesCount, uint32_t* storageInitError);

/* 自动保存数据（如果与上次调用不同）
 * 函数功能：周期性调用以自动保存数据。每次调用时验证数据的 CRC 校验和是否与上次的
 *         校验和不同。如果不同，则将数据保存到预先打开的文件中
 * 使用说明：应该由程序周期性调用
 * 参数说明：
 *   - storage: 存储对象
 *   - closeFiles: 如果为 true，则关闭所有文件。在程序结束时使用
 * 返回值说明：成功返回 0，或返回 subIndexOD 值的位掩码，表示无法保存数据的条目
 */
/**
 * Automatically save data if differs from previous call.
 *
 * Should be called cyclically by program. Each interval it verifies, if crc checksum of data differs from previous
 * checksum. If it does, data are saved into pre-opened file.
 *
 * @param storage This object
 * @param closeFiles If true, then all files will be closed. Use on end of the program.
 *
 * @return 0 on success or bit mask from subIndexOD values, where data was not able to be saved.
 */
uint32_t CO_storageLinux_auto_process(CO_storage_t* storage, bool_t closeFiles);

/** @} */ /* CO_storageLinux */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */

#endif /* CO_STORAGE_LINUX_H */
