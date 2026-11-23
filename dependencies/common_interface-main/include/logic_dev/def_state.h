/**
 * @file def_state.h
 * @author Harvey (Harvey.Wang@dayepower.com)
 * @brief state definitions based on ROS2 service messages
 * @version 0.2
 * @date 2026-06-17
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved
 *
 */

#pragma once

#include <cstdint>

// ============================================================
// 通用成功/失败状态 (ToAppCmd.srv status)
// ============================================================
enum class ResultStatus {
  SUCCESS = 1,  // 成功
  FAILURE = 2,  // 失败
};

// ============================================================
// 导航状态 (GetNaviStatus.srv) - 9种状态
// ============================================================
enum class NaviStatus {
  NOT_READY            = 0,  // 未就绪（未加载图层，系统未就绪，不可作业）
  STANDBY              = 1,  // 待机（图层已加载，系统就绪，可随时接收作业指令）
  LOADING_LAYER        = 2,  // 加载图层中
  EDGE_MOWING          = 3,  // 沿边割草中
  FULL_COVERAGE_MOWING = 4,  // 全覆盖割草中
  STOPPING_TASK        = 5,  // 停止任务中
  RETURNING_DOCK       = 6,  // 回充中
  LEAVING_DOCK         = 7,  // 退基站中
  TRANSITING           = 8,  // 转场中
};

// ============================================================
// SLAM状态 (GetSlamStatus.srv)
// ============================================================
enum class SlamStatus {
  UNINITIALIZED = 0,  // 未初始化
  INITIALIZING  = 1,  // 初始化中
  WORKING       = 2,  // 工作中
};

// ============================================================
// 感知状态 (GetPercepStatus.srv)
// ============================================================
enum class PercepStatus {
  UNKNOWN        = -1,  // 未知状态
  STANDBY        = 0,   // 待机中
  PUB_MAP        = 1,   // 感知地图发布中
  PUB_QRCODE     = 2,   // 基站二维码发布中
};

// ============================================================
// Camera 摄像头状态 (ToCameraCmd.srv)
// ============================================================
enum class CameraStatus {
  UNKNOWN = -1,  // 未知状态
  CLOSED  = 0,   // 关闭
  OPENED  = 1,   // 打开
};

// ============================================================
// ToAppCmd 指令定义 (ToAppCmd.srv) - 33种情况
// ============================================================
enum class ToAppCmdType {
  SLAM_MAP_BUILD_DONE        = 1,     // SLAM通知平台：建图完成
  SLAM_LAYER_SAVE_DONE       = 20,    // SLAM通知平台：图层保存完成
  SLAM_LAYER_DELETE_DONE     = 21,    // SLAM通知平台：删除图层完成
  SLAM_MAP_SAVE_DONE         = 30,    // SLAM通知平台：地图保存完成
  SLAM_MAP_DELETE_DONE       = 31,    // SLAM通知平台：删除地图完成
  SLAM_LOC_START_DONE        = 40,    // SLAM通知平台：开启定位完成
  SLAM_LOC_STOP_DONE         = 41,    // SLAM通知平台：停止定位完成
  SLAM_STOP_MAPPING_EVENT    = 50,    // SLAM通知平台：终止建图事件
  SLAM_ISOLATED_ISLAND_EVENT = 51,    // SLAM通知平台：孤岛提示事件
  SLAM_AREA_SIZE_EVENT       = 52,    // SLAM通知平台：区域大小提示事件
  SLAM_MULTI_CHANNEL_EVENT   = 53,    // SLAM通知平台：多通道提示事件
  SLAM_RELOC_PREPARE_DONE    = 60,    // SLAM通知平台：重定位准备工作完成
  SLAM_RELOC_EXECUTE_DONE    = 61,    // SLAM通知平台：重定位执行完成
  SLAM_RELOC_STOP_DONE       = 62,    // SLAM通知平台：重定位停止完成
  SLAM_MAPPING_ENDED         = 100,   // SLAM通知平台：建图被结束
  NAVI_LEAVE_DOCK_DONE       = 1001,  // 导航通知平台：退基站完成
  NAVI_TRANSIT_DONE          = 1010,  // 导航通知平台：转场完成
  NAVI_LAYER_LOAD_DONE       = 1020,  // 导航通知平台：图层加载完成
  NAVI_CLEAR_TRACK_DONE      = 1021,  // 导航通知平台：清除割草轨迹完成(地图被删除)
  NAVI_EDGE_MOW_DONE         = 1040,  // 导航通知平台：沿边割草完成
  NAVI_BOW_MOW_DONE          = 1041,  // 导航通知平台：弓字割草完成
  NAVI_START_TASK_DONE       = 1050,  // 导航通知平台：割草新任务准备完成
  NAVI_PAUSE_DONE            = 1060,  // 导航通知平台：暂停割草任务完成
  NAVI_RESUME_DONE           = 1061,  // 导航通知平台：继续割草任务完成
  NAVI_RELOC_START_DONE      = 1070,  // 导航通知平台：开始执行重定位动作完成
  NAVI_RELOC_STOP_DONE       = 1071,  // 导航通知平台：停止执行重定位动作完成
  NAVI_STOP_TASK_DONE        = 1100,  // 导航通知平台：停止割草任务完成
  NAVI_STOP_DOCK_DONE        = 1101,  // 导航通知平台：停止回充任务完成
  NAVI_RETURN_DOCK_DONE      = 1200,  // 导航通知平台：回充任务完成
  NAVI_POWER_OFF_DOCK_DONE   = 1201,  // 导航通知平台：掉电对接基站任务完成
  PERCEP_PUB_MAP_DONE        = 2001,  // 感知通知平台：感知地图发布模块启动完成
  PERCEP_PUB_QRCODE_DONE     = 2002,  // 感知通知平台：基站二维码发布模块启动完成
  PERCEP_PAUSE_DONE          = 2021,  // 感知通知平台：停止工作完成
};

// ============================================================
// ToNaviCmd 指令定义
// ============================================================
enum class ToNaviCmdType {
  LEAVE_DOCK         = 1,    // 导航退基站
  TRANSIT            = 10,   // 导航开始转场
  LOAD_LAYER         = 20,   // 导航加载图层
  CLEAR_TRACK        = 21,   // 地图被删除, 导航清除割草轨迹
  EDGE_MOW           = 40,   // 导航沿边割草
  BOW_MOW            = 41,   // 导航弓字割草
  SET_CUT_HEIGHT     = 42,   // 导航实时修改割草高度
  START_TASK         = 50,   // 导航割草新任务准备
  PAUSE_TASK         = 60,   // 导航暂停割草任务
  RESUME_TASK        = 61,   // 导航继续割草任务
  START_RELOCALIZATION = 70, // 导航开始执行重定位动作
  STOP_RELOCALIZATION  = 71, // 导航停止执行重定位动作
  STOP_WORK          = 100,  // 导航停止割草任务
  STOP_RETURN_DOCK   = 101,  // 导航停止回充任务
  RETURN_DOCK        = 200,  // 导航开始回充任务
  POWER_OFF_DOCK     = 201,  // 导航开始掉电对接基站任务
};

// ============================================================
// ToSlamCmd 指令定义 (ToSlamCmd.srv)
// ============================================================
enum class ToSlamCmdType {
  START_MAPPING           = 1,    // SLAM开始建图
  SET_OUTER_CONTOUR_START = 2,    // SLAM设置外轮廓起点
  CLOSE_CONTOUR           = 20,   // SLAM闭合轮廓
  DELETE_AREA             = 21,   // SLAM删除区域
  SET_CHANNEL_COVER       = 22,   // SLAM设置通道覆盖指令
  SAVE_MAP                = 30,   // SLAM保存地图
  DELETE_MAP              = 31,   // SLAM删除地图
  START_LOCALIZATION      = 40,   // SLAM开启定位
  STOP_LOCALIZATION       = 41,   // SLAM停止定位
  START_RELOCALIZATION    = 61,   // SLAM执行重定位
  STOP_RELOCALIZATION     = 62,   // SLAM停止重定位
  END_MAPPING             = 100,  // SLAM结束建图
};

// ============================================================
// ToPercepCmd 指令定义
// ============================================================
enum class ToPercepCmdType {
  PUB_MAP        = 1,   // 感知启动感知地图发布模块
  PUB_QRCODE     = 2,   // 感知启动基站二维码发布模块
  STOP_WORK      = 21,  // 感知停止工作，进入待机
};

// ============================================================
// ToCameraCmd 指令定义
// ============================================================
enum class ToCameraCmdType {
  QUERY_STATUS  = 1,  // 查询摄像头工作状态
  REQUEST_OPEN  = 2,  // 请求打开摄像头
  REQUEST_CLOSE = 3,  // 请求关闭摄像头
};
