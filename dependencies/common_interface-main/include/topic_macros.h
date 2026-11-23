/**
 * @file topic_macros.h
 * @author Harry (Harry.Wu@dayepower.com)
 * @brief topic name macros
 * @version 0.1
 * @date 2026-06-04
 *
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved
 *
 */
#pragma once
// 控制
inline constexpr const char *kMotorSet = "/mcu/motor_set";
inline constexpr const char *kLightCtrl = "/mcu/light_ctrl";
inline constexpr const char *kMcuCtrl = "/mcu/status_ctrl";

// 传感器上报
inline constexpr const char *kMotorInfo = "/mcu/motor_info";
inline constexpr const char *kImu = "/mcu/imu";
inline constexpr const char *kOdo = "/mcu/odo";
inline constexpr const char *kBat = "/mcu/bat";
inline constexpr const char *kSignal = "/mcu/signal";
inline constexpr const char *kButtonKey = "/mcu/button_key";
inline constexpr const char *kButtonSignal = "/mcu/button_signal";

// 感知
inline constexpr const char *kGridMapInfo = "/percep_demo/grid_map";
inline constexpr const char *kCameraParam = "/camera/params";

// 割草任务相关的srv名定义
inline constexpr const char *kToSlamCmd = "/to_slam_cmd";
inline constexpr const char *kGetSlamStatus = "/get_slam_status";
inline constexpr const char *kToNaviCmd = "/to_navi_cmd";
inline constexpr const char *kGetNaviStatus = "/get_navi_status";
inline constexpr const char *kToPercepCmd = "/to_percep_cmd";
inline constexpr const char *kGetPercepStatus = "/get_percep_status";
inline constexpr const char *kToApplicationCmd = "/to_application_cmd";
inline constexpr const char *kToCameraCmd = "/to_camera_cmd";
inline constexpr const char *kGetWorkMap = "/get_work_map";

// iot相关数据
inline constexpr const char *kCustomUp = "/iot/custom_msg/up";
inline constexpr const char *kCustomSyncDown = "/iot/custom_msg/sync_down";
inline constexpr const char *kCustomAsyncDown = "/iot/custom_msg/async_down";
inline constexpr const char *kIoTMapDataUp = "/iot/map_data/up";
inline constexpr const char *kIoTIncrementalTrackUp = "/iot/incremental_track/up";
inline constexpr const char *kIoTSlamFullTrackUp = "/iot/slam_full_track/up";
inline constexpr const char *kIoTNaviFullTrackUp = "/iot/navi_full_track/up";
inline constexpr const char *kIoTLocationUp = "/iot/location/up";
// biz
inline constexpr const char *kBizGetMapTrajectory = "/biz/get_map_trajectory";
inline constexpr const char *kBizConfigUpdate = "/biz/config_update";
inline constexpr const char *kBizStartMapping = "/biz/start_mapping";
inline constexpr const char *kBizStopMapping = "/biz/stop_mapping";
inline constexpr const char *kBizEndCreateMap = "/biz/end_create_map";
inline constexpr const char *kBizCompleteMapping = "/biz/complete_mapping";
inline constexpr const char *kBizStartMowing = "/biz/start_mowing";
inline constexpr const char *kBizDispatchScheduledTask = "/biz/dispatch_scheduled_task";
inline constexpr const char *kBizStopMowing = "/biz/stop_mowing";
inline constexpr const char *kBizTaskEvent = "/biz/task_event";
inline constexpr const char *kBizOnAppCmd = "/biz/on_app_cmd";
inline constexpr const char *kBizAppControl = "/biz/app_control";
inline constexpr const char *kBizSetContourOrigin = "/biz/set_contour_origin";
inline constexpr const char *kBizSetCounterClosed = "/biz/set_counter_closed";
inline constexpr const char *kBizPropertyDown = "/biz/property_down";
inline constexpr const char *kBizAreaConfigUpdate = "/biz/area_config_update";
inline constexpr const char *kBizSetAreaConfig = "/biz/set_area_config";
inline constexpr const char *kBizDeleteMap = "/biz/delete_map";
inline constexpr const char *kBizDeleteMapComplete = "/biz/delete_map_complete";
inline constexpr const char *kBizSetChannelOverride = "/biz/set_channel_override";
inline constexpr const char *kBizStartCharging = "/biz/start_charging";
inline constexpr const char *kBizStopCharging = "/biz/stop_charging";
inline constexpr const char *kBizStopWorking = "/biz/stop_working";
inline constexpr const char *kBizPauseCharge = "/biz/pause_charge";
inline constexpr const char *kBizResumeCharge = "/biz/resume_charge";
inline constexpr const char *kBizSetCreateLayerProperty = "/biz/set_create_layer_property";
inline constexpr const char *kBizOtaStartCommand = "/biz/ota/start_command";
inline constexpr const char *kBizOtaUserConfirm = "/biz/ota/user_confirm";
inline constexpr const char *kBizOtaInternalResult = "/biz/ota/internal_result";
inline constexpr const char *kBizNetwork = "/biz/network";
inline constexpr const char *kBizHealth = "/biz/health";
inline constexpr const char *kBizHealthTop = "/biz/health_top";
inline constexpr const char *kBizHealthList = "/biz/health_list";
inline constexpr const char *kBizHealthAffected = "/biz/health_affected";
inline constexpr const char *kBizDeviceReboot = "/biz/device_reboot";
inline constexpr const char *kBizButtonOk = "/biz/button_ok";
inline constexpr const char *kBizGetDeviceStatus = "/biz/get_device_status";
inline constexpr const char *kBizStartLogUpload = "/biz/start_log_upload";
inline constexpr const char *kBizPauseMowing = "/biz/pause_mowing";
inline constexpr const char *kBizResumeMowing = "/biz/resume_mowing";
inline constexpr const char *kBizStartGlobalMowing = "/biz/start_global_mowing";
inline constexpr const char *kBizChooseMappingMethod = "/biz/choose_mapping_method";
inline constexpr const char *kBizResumeUninterruptedTask = "/biz/resume_uninterrupted_task";
inline constexpr const char *kBizTaskResult = "/biz/task_result";
inline constexpr const char *kBizFactoryPreCheck = "/biz/factory_pre_check";
inline constexpr const char *kBizStartRemoteMowing = "/biz/start_remote_mowing";
inline constexpr const char *kBizStopRemoteMowing = "/biz/stop_remote_mowing";
inline constexpr const char *kBizSetCutterSwitch = "/biz/set_cutter_switch";
inline constexpr const char *kBizFactoryReset = "/biz/factory_reset";
inline constexpr const char *kBizDisturb = "/biz/disturb";
inline constexpr const char *kBizRainProtection = "/biz/rain_protection";
inline constexpr const char *kBizSetPinCode = "/biz/set_pin_code";
inline constexpr const char *kBizDeleteZone = "/biz/delete_zone";
inline constexpr const char *kBizMergeZone = "/biz/merge_zone";
inline constexpr const char *kBizSplitZone = "/biz/split_zone";
inline constexpr const char *kBizCancelZoneSplit = "/biz/cancel_zone_split";
inline constexpr const char *kBizSetSplitPoint = "/biz/set_split_point";
inline constexpr const char *kBizBackupMap = "/biz/backup_map";
inline constexpr const char *kBizRetrieveMap = "/biz/retrieve_map";
inline constexpr const char *kBizRestoreMap = "/biz/restore_map";
inline constexpr const char *kBizFindDevice = "/biz/find_device";
inline constexpr const char *kBizMapDataUpdated = "/biz/map_data_updated";
inline constexpr const char *kBizDeleteChannel = "/biz/delete_channel";
inline constexpr const char *kBizMapSelfCheck = "/biz/map_self_check";

// slam
inline constexpr const char *kSlamMapBytes = "/slam_map_bytes";
inline constexpr const char *kSlam2DPose = "/slam_2d_pose";
inline constexpr const char *kRequestFullMap = "/request_fullmap";
inline constexpr const char *kAllWorkMap = "/all_work_map";
inline constexpr const char *kAllTrackPoint = "/all_track_point";
inline constexpr const char *kAllCuttingPathPoint = "/all_cutting_path_point";
inline constexpr const char *kSimpleZones = "/simple_zones";

// rtk
inline constexpr const char *kRtkGga = "/m720/gga";  // ros-humble-nmea-msgs
inline constexpr const char *kRtkRtcm = "/m720/rtcm";  // ros-humble-rtcm-msgs

// navi
inline constexpr const char *kNaviMowingStatus = "/navi/mowing_status";
inline constexpr const char *kNaviMapBytes = "/navi_map_bytes";
inline constexpr const char *kNaviRequestTrajectory = "/navi/request_trajectory";
inline constexpr const char *kNavRemoteCtrl = "/to_nav_remote_ctrl";

// ota
inline constexpr const char *kMcuOtaCmd = "/mcu/ota_cmd";
inline constexpr const char *kMcuOtaStatus = "/mcu/ota_status";
inline constexpr const char *kMcuVersion = "/mcu/version";
inline constexpr const char *kRTKOtaCmd = "/m720/firmware_upgrade";
inline constexpr const char *kRTKOtaStatus = "/m720/firmware_upgrade_progress";
inline constexpr const char *kLidarOtaCmd = "/lidar/firmware_upgrade";
inline constexpr const char *kLidarOtaStatus = "/lidar/firmware_upgrade_progress";

// shm
inline constexpr const char *kMapData = "mapData";  //4M
inline constexpr const char *kSlamFullTrack = "slamFullTrack";  //6M
inline constexpr const char *kNaviFullTrack = "naviFullTrack";  //10M
