/**
 * @file file_path.h
 * @author Harvey (Harvey.Wang@dayepower.com)
 * @brief file path macro definitions
 * @version 0.2
 * @date 2026-06-17
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved
 *
 */

#pragma once

#include <string>

// 基础路径定义
#define BACKUP_PATH              "/data/hj/config/"
#ifdef __aarch64__
#define LOGIC_DIR                "/userdata/logic/"
#else
#define LOGIC_DIR                "/tmp/logic/"
#endif

// 示例：拼接路径宏（其他同类路径宏按此模式自行添加）
#define PARA_CONFIG_FILE_PATH         LOGIC_DIR    "paraConfig.json"

// 示例：常量
const int32_t CLEAN_RECORD_MAX_DAYS = 28;

// 示例：条件编译的路径
#ifdef __aarch64__
constexpr const char* kRemoteDir = "/userdata/remote";
constexpr const char* kReturnDockFailedFlagPath = "/tmp/returning_to_dock_failed.flag";
constexpr const char* kUninterruptedTaskFlagPath = "/userdata/log/uninterrupted_task.flag";
// 重定位标志: 存在=不需要重定位, 不存在=需要重定位.
constexpr const char* kRelocalizationFlagPath = "/tmp/relocalization.flag";
// 行为树 XML: 调试目录存在时优先使用, 否则使用默认(固件)目录.
constexpr const char* kBtXmlDir = "/app/daye/config/biz_node/bt";
constexpr const char* kBtXmlDebugDir = "/userdata/config/bt";
// 故障码响应策略: 调试目录存在时优先使用, 否则使用默认(固件)目录.
constexpr const char* kFaultPolicyDir = "/app/daye/config/biz_node/fault";
constexpr const char* kFaultPolicyDebugDir = "/userdata/config/fault";

#else
constexpr const char* kRemoteDir = "/tmp/remote";
constexpr const char* kReturnDockFailedFlagPath = "/tmp/returning_to_dock_failed.flag";
constexpr const char* kUninterruptedTaskFlagPath = "/tmp/uninterrupted_task.flag";
// 重定位标志: 存在=不需要重定位, 不存在=需要重定位.
constexpr const char* kRelocalizationFlagPath = "/tmp/relocalization.flag";
// 行为树 XML: 调试目录存在时优先使用, 否则使用默认目录.
constexpr const char* kBtXmlDir = "/home/jsj50038/workspace/config/bt";
constexpr const char* kBtXmlDebugDir = "/tmp/bt";
// 故障码响应策略: 调试目录存在时优先使用, 否则使用默认目录.
constexpr const char* kFaultPolicyDir = "/home/jsj50038/workspace/config/fault";
constexpr const char* kFaultPolicyDebugDir = "/tmp/fault";
#endif

// 示例：功能开关宏
#define FOR_MULTI_PLATFORM_CLEAN

// 示例：std::string 常量
const std::string NORMAL_RUN = "normal";
