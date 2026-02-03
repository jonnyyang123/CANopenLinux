/*
 * CANopen data storage object for Linux
 *
 * @file        CO_storageLinux.c
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

#include "CO_storageLinux.h"
#include "301/crc16-ccitt.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

/* 函数功能: 在"存储参数"命令时写入数据到文件 - OD对象1010
 * 执行步骤:
 *   步骤1: 创建临时文件名和旧文件名
 *   步骤2: 打开临时文件并写入数据和CRC校验和
 *   步骤3: 验证写入的数据: 读取临时文件并检查长度和CRC
 *   步骤4: 将现有文件重命名为.old，临时文件重命名为正式文件
 *   步骤5: 清理内存资源
 * 参数说明:
 *   - entry: 存储条目指针，包含文件名、地址和长度信息
 *   - CANmodule: CAN模块指针(本函数未使用)
 * 返回值说明: 返回ODR_t类型错误码(ODR_OK成功，ODR_OUT_OF_MEM内存不足，ODR_HW硬件错误)
 */
/*
 * Function for writing data on "Store parameters" command - OD object 1010
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t
storeLinux(CO_storage_entry_t* entry, CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    ODR_t ret = ODR_OK;
    uint16_t crc_store;

    /* 创建临时文件名和旧文件备份名 */
    /* Create names for temporary and old file */
    size_t fn_len = strlen(entry->filename) + 5;
    char* filename_tmp = malloc(fn_len);
    char* filename_old = malloc(fn_len);
    /* 内存分配失败处理 */
    if (filename_tmp == NULL || filename_old == NULL) {
        if (filename_tmp != NULL) {
            free(filename_tmp);
        }
        if (filename_old != NULL) {
            free(filename_old);
        }
        ret = ODR_OUT_OF_MEM;
    } else {
        strcpy(filename_tmp, entry->filename);
        strcpy(filename_old, entry->filename);
        strcat(filename_tmp, ".tmp");
        strcat(filename_old, ".old");
    }

    /* 打开临时文件并写入数据 */
    /* Open a temporary file and write data to it */
    if (ret == ODR_OK) {
        FILE* fp = fopen(filename_tmp, "w");
        if (fp == NULL) {
            ret = ODR_HW;
        } else {
            /* 写入数据和CRC: 注意以下两行存在竞态条件，但由于该函数仅被SDO服务器调用，已通过CO_LOCK_OD保护 */
            /* following two lines are subject to race conditions. This function is called only by SDO server
             * and so it is already protected by CO_LOCK_OD. */
            size_t cnt = fwrite(entry->addr, 1, entry->len, fp);
            crc_store = crc16_ccitt(entry->addr, entry->len, 0);
            cnt += fwrite(&crc_store, 1, sizeof(crc_store), fp);
            fclose(fp);
            if (cnt != (entry->len + sizeof(crc_store))) {
                ret = ODR_HW;
            }
        }
    }

    /* 验证写入的数据 */
    /* Verify data */
    if (ret == ODR_OK) {
        uint8_t* buf = NULL;
        FILE* fp = NULL;
        size_t cnt = 0;
        uint16_t crc_verify, crc_read;

        buf = malloc(entry->len + 4);
        if (buf != NULL) {
            fp = fopen(filename_tmp, "r");
            if (fp != NULL) {
                cnt = fread(buf, 1, entry->len + 4, fp);
                crc_verify = crc16_ccitt(buf, entry->len, 0);
                fclose(fp);
                memcpy(&crc_read, &buf[entry->len], sizeof(crc_read));
            }
            free(buf);
        }
        /* 如果大小或CRC不匹配，报告错误 */
        /* If size or CRC differs, report error */
        if (buf == NULL || fp == NULL || cnt != (entry->len + sizeof(crc_verify)) || crc_store != crc_verify
            || crc_store != crc_read) {
            ret = ODR_HW;
        }
    }

    /* 将现有文件重命名为*.old，临时文件重命名为正式文件名 */
    /* rename existing file to *.old and *.tmp to existing */
    if (ret == ODR_OK) {
        rename(entry->filename, filename_old);
        if (rename(filename_tmp, entry->filename) != 0) {
            ret = ODR_HW;
        }
    }

    if (ret != ODR_OUT_OF_MEM) {
        free(filename_tmp);
        free(filename_old);
    }

    return ret;
}

/* 函数功能: 恢复默认参数 - OD对象1011
 * 执行步骤:
 *   步骤1: 如果是自动存储模式，先关闭已打开的文件
 *   步骤2: 创建.old后缀的备份文件名
 *   步骤3: 将现有文件重命名为备份文件
 *   步骤4: 创建新的空文件并写入"-\n"标记(表示使用默认值)
 * 参数说明:
 *   - entry: 存储条目指针
 *   - CANmodule: CAN模块指针(本函数未使用)
 * 返回值说明: 返回ODR_t类型错误码(ODR_OK成功，ODR_OUT_OF_MEM内存不足，ODR_HW硬件错误)
 */
/*
 * Function for restoring data on "Restore default parameters" command - OD 1011
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t
restoreLinux(CO_storage_entry_t* entry, CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
    ODR_t ret = ODR_OK;

    /* 如果是自动存储模式，先关闭文件 */
    /* close the file first, if auto storage */
    if ((entry->attr & CO_storage_auto) != 0 && entry->fp != NULL) {
        fclose(entry->fp);
        entry->fp = NULL;
    }

    /* 将现有文件重命名为*.old */
    /* Rename existing filename to *.old. */
    char* filename_old = malloc(strlen(entry->filename) + 5);
    if (filename_old == NULL) {
        ret = ODR_OUT_OF_MEM;
    } else {
        strcpy(filename_old, entry->filename);
        strcat(filename_old, ".old");
        rename(entry->filename, filename_old);
        free(filename_old);
    }

    /* 创建空文件并写入"-\n"标记 */
    /* create an empty file and write "-\n" to it. */
    if (ret == ODR_OK) {
        FILE* fp = fopen(entry->filename, "w");
        if (fp == NULL) {
            ret = ODR_HW;
        } else {
            fputs("-\n", fp);
            fclose(fp);
        }
    }

    return ret;
}

/* 函数功能: 初始化Linux平台的CANopen存储系统(重要函数)
 * 执行步骤:
 *   步骤1: 验证所有输入参数的有效性
 *   步骤2: 初始化存储对象和OD扩展，注册存储和恢复回调函数
 *   步骤3: 遍历所有存储条目进行初始化
 *   步骤4: 对每个条目: 打开文件，读取数据，验证CRC校验和
 *   步骤5: 如果数据有效，复制到目标地址; 如果无效，使用默认值
 *   步骤6: 对于自动存储条目，保持文件打开以供后续自动保存
 *   步骤7: 记录所有初始化错误到storageInitError位图
 * 参数说明:
 *   - storage: 存储对象指针
 *   - CANmodule: CAN模块指针
 *   - OD_1010_StoreParameters: 存储参数OD条目(1010h)
 *   - OD_1011_RestoreDefaultParam: 恢复默认参数OD条目(1011h)
 *   - entries: 存储条目数组指针
 *   - entriesCount: 存储条目数量
 *   - storageInitError: 初始化错误位图指针
 * 返回值说明: 返回CO_ReturnError_t错误码
 */
CO_ReturnError_t
CO_storageLinux_init(CO_storage_t* storage, CO_CANmodule_t* CANmodule, OD_entry_t* OD_1010_StoreParameters,
                     OD_entry_t* OD_1011_RestoreDefaultParam, CO_storage_entry_t* entries, uint8_t entriesCount,
                     uint32_t* storageInitError) {
    CO_ReturnError_t ret;

    /* 验证参数有效性 */
    /* verify arguments */
    if (storage == NULL || entries == NULL || entriesCount == 0 || storageInitError == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    storage->enabled = false;

    /* 初始化存储对象和OD扩展 */
    /* initialize storage and OD extensions */
    ret = CO_storage_init(storage, CANmodule, OD_1010_StoreParameters, OD_1011_RestoreDefaultParam, storeLinux,
                          restoreLinux, entries, entriesCount);
    if (ret != CO_ERROR_NO) {
        return ret;
    }

    /* 初始化所有存储条目 */
    /* initialize entries */
    *storageInitError = 0;
    for (uint8_t i = 0; i < entriesCount; i++) {
        CO_storage_entry_t* entry = &entries[i];
        bool_t dataCorrupt = false;
        char* writeFileAccess = "w";

        /* 验证条目参数 */
        /* verify arguments */
        if (entry->addr == NULL || entry->len == 0 || entry->subIndexOD < 2 || strlen(entry->filename) == 0) {
            *storageInitError = i;
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }

        /* 打开文件，检查是否存在并创建临时缓冲区 */
        /* Open file, check existence and create temporary buffer */
        uint8_t* buf = NULL;
        FILE* fp = fopen(entry->filename, "r");
        if (fp == NULL) {
            dataCorrupt = true;
            ret = CO_ERROR_DATA_CORRUPT;
        } else {
            buf = malloc(entry->len + sizeof(uint16_t));
            if (buf == NULL) {
                fclose(fp);
                *storageInitError = i;
                return CO_ERROR_OUT_OF_MEMORY;
            }
        }

        /* 先读取数据到临时缓冲区，然后验证并复制到目标地址 */
        /* Read data into temporary buffer first. Then verify and copy to addr */
        if (!dataCorrupt) {
            size_t cnt = fread(buf, 1, entry->len + sizeof(uint16_t), fp);

            /* 如果文件为空(包含"-"标记)，跳过加载，使用默认值，不报错；否则验证长度和CRC后复制数据 */
            /* If file is empty, just skip loading, default values will be used,
             * no error. Otherwise verify length and crc and copy data. */
            if (!(cnt == 2 && buf[0] == '-')) {
                uint16_t crc1, crc2;
                crc1 = crc16_ccitt(buf, entry->len, 0);
                memcpy(&crc2, &buf[entry->len], sizeof(crc2));

                if (crc1 == crc2 && cnt == (entry->len + sizeof(crc2))) {
                    memcpy(entry->addr, buf, entry->len);
                    entry->crc = crc1;
                    writeFileAccess = "r+";
                } else {
                    dataCorrupt = true;
                    ret = CO_ERROR_DATA_CORRUPT;
                }
            }

            free(buf);
            fclose(fp);
        }

        /* 错误情况下的附加信息 */
        /* additional info in case of error */
        if (dataCorrupt) {
            uint32_t errorBit = entry->subIndexOD;
            if (errorBit > 31) {
                errorBit = 31;
            }
            *storageInitError |= ((uint32_t)1) << errorBit;
        }

        /* 如果设置了自动存储，打开文件以供后续使用 */
        /* open file for auto storage, if set so */
        if ((entry->attr & CO_storage_auto) != 0) {
            entry->fp = fopen(entry->filename, writeFileAccess);
            if (entry->fp == NULL) {
                *storageInitError = i;
                return CO_ERROR_ILLEGAL_ARGUMENT;
            }
        }
    } /* 所有条目初始化完成 for (entries) */

    /* 启用存储功能 */
    storage->enabled = true;
    return ret;
}

/* 函数功能: 自动存储处理函数 - 定期保存变化的数据(重要函数)
 * 执行步骤:
 *   步骤1: 验证存储对象指针有效性
 *   步骤2: 遍历所有存储条目
 *   步骤3: 跳过非自动存储条目或文件未打开的条目
 *   步骤4: 计算当前数据的CRC校验和
 *   步骤5: 如果CRC与上次保存的不同，说明数据已变化
 *   步骤6: 将文件指针重置到开头，写入新数据和CRC
 *   步骤7: 刷新文件缓冲区确保数据写入磁盘
 *   步骤8: 如果写入失败，设置错误位
 *   步骤9: 如果需要，关闭所有文件
 * 参数说明:
 *   - storage: 存储对象指针
 *   - closeFiles: 是否关闭文件的布尔标志
 * 返回值说明: 返回错误位图，每位对应一个条目的存储状态
 */
uint32_t
CO_storageLinux_auto_process(CO_storage_t* storage, bool_t closeFiles) {
    uint32_t storageError = 0;

    /* 验证参数 */
    /* verify arguments */
    if (storage == NULL) {
        return false;
    }

    /* 遍历所有存储条目 */
    /* loop through entries */
    for (uint8_t i = 0; i < storage->entriesCount; i++) {
        CO_storage_entry_t* entry = &storage->entries[i];

        /* 跳过非自动存储条目 */
        if ((entry->attr & CO_storage_auto) == 0 || entry->fp == NULL) {
            continue;
        }

        /* 如果当前数据的CRC与保存的不同，则保存文件 */
        /* If CRC of the current data differs, save the file */
        uint16_t crc = crc16_ccitt(entry->addr, entry->len, 0);
        if (crc != entry->crc) {
            size_t cnt;
            /* 重置文件指针到开头 */
            rewind(entry->fp);
            /* 在OD锁保护下写入数据 */
            CO_LOCK_OD(storage->CANmodule);
            cnt = fwrite(entry->addr, 1, entry->len, entry->fp);
            CO_UNLOCK_OD(storage->CANmodule);
            /* 写入CRC校验和 */
            cnt += fwrite(&crc, 1, sizeof(crc), entry->fp);
            /* 刷新文件缓冲区 */
            fflush(entry->fp);
            if (cnt == (entry->len + sizeof(crc))) {
                /* 更新保存的CRC值 */
                entry->crc = crc;
            } else {
                /* 保存失败，记录错误 */
                /* error with save */
                uint32_t errorBit = entry->subIndexOD;
                if (errorBit > 31) {
                    errorBit = 31;
                }
                storageError |= ((uint32_t)1) << errorBit;
            }
        }

        /* 如果需要，关闭文件 */
        if (closeFiles) {
            fclose(entry->fp);
            entry->fp = NULL;
        }
    }

    return storageError;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
