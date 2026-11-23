/**
 * @file machine_state.h
 * @author Harvey (Harvey.Wang@dayepower.com)
 * @brief state definition for the machine and task
 * @version 0.1
 * @date 2026-06-05
 * 
 * Copyright 2026 Daye Technology Co.Ltd. All rights reserved
 * 
 */

#pragma once

#include "enum.h"
#include <string>
#include <unordered_map>

namespace machine {

smart_enum_class(MachineState,
                    Booting,                // 启动中
                    StandingBy,             // 待机中
                    OtaUpdating,            // OTA更新
                    Charging,               // 充电中
                    ReturningToDock,        // 回充中
                    Mapping,                // 构建地图
                    Mowing,                 // 割草中
                    RemoteMowing,           // 遥控割草
                    Alarm,                  // 报警
                    NetworkProvisioning,    // 配网中
                    FactoryTest,            // 工厂测试中
                    FactoryReset,           // 恢复出厂设置
                    Shutdown,               // 关机
                    LeavingDock             // 退基站中
);

smart_enum_class(MachineEvent,
                    ToMapping,                // 建图事件
                    ToMowing,                 // 割草事件
                    ToRemoteMowing,           // 遥控割草事件
                    ToShutdown,               // 关机事件
                    ToOtaUpdating,            // OTA事件
                    ToCharging,               // 充电事件
                    ToStandby,                // 待机事件
                    ToReturningToDock,        // 回充事件
                    ToAlarm,                  // 报警事件
                    ToNetworkProvisioning,    // 配网事件
                    ToFactoryTest,            // 工厂测试事件
                    ToPaused,                 // 暂停事件
                    ToResume,                 // 恢复事件
                    ToStopped,                // 停止事件
                    ToFactoryReset,           // 恢复出厂设置事件
                    ToLeavingDock             // 退基站事件
);

static std::unordered_map<MachineState, std::string> state_enum_to_str_map = {
    {MachineState::Booting, "Booting"},
    {MachineState::StandingBy, "StandingBy"},
    {MachineState::OtaUpdating, "OtaUpdating"},
    {MachineState::Charging, "Charging"},
    {MachineState::ReturningToDock, "ReturningToDock"},
    {MachineState::Mapping, "Mapping"},
    {MachineState::Mowing, "Mowing"},
    {MachineState::RemoteMowing, "RemoteMowing"},
    {MachineState::Alarm, "Alarm"},
    {MachineState::NetworkProvisioning, "NetworkProvisioning"},
    {MachineState::FactoryTest, "FactoryTest"},
    {MachineState::FactoryReset, "FactoryReset"},
    {MachineState::Shutdown, "Shutdown"},
    {MachineState::LeavingDock, "LeavingDock"}
};

static std::unordered_map<MachineEvent, std::string> event_enum_to_str_map = {
    {MachineEvent::ToMapping, "ToMapping"},
    {MachineEvent::ToMowing, "ToMowing"},
    {MachineEvent::ToRemoteMowing, "ToRemoteMowing"},
    {MachineEvent::ToShutdown, "ToShutdown"},
    {MachineEvent::ToOtaUpdating, "ToOtaUpdating"},
    {MachineEvent::ToCharging, "ToCharging"},
    {MachineEvent::ToStandby, "ToStandby"},
    {MachineEvent::ToReturningToDock, "ToReturningToDock"},
    {MachineEvent::ToAlarm, "ToAlarm"},
    {MachineEvent::ToNetworkProvisioning, "ToNetworkProvisioning"},
    {MachineEvent::ToFactoryTest, "ToFactoryTest"},
    {MachineEvent::ToPaused, "ToPaused"},
    {MachineEvent::ToResume, "ToResume"},
    {MachineEvent::ToStopped, "ToStopped"},
    {MachineEvent::ToFactoryReset, "ToFactoryReset"},
    {MachineEvent::ToLeavingDock, "ToLeavingDock"}
};

static std::unordered_map<std::string, MachineState> state_str_to_enum_map = {
    {"Booting", MachineState::Booting},
    {"StandingBy", MachineState::StandingBy},
    {"OtaUpdating", MachineState::OtaUpdating},
    {"Charging", MachineState::Charging},
    {"ReturningToDock", MachineState::ReturningToDock},
    {"Mapping", MachineState::Mapping},
    {"Mowing", MachineState::Mowing},
    {"RemoteMowing", MachineState::RemoteMowing},
    {"Alarm", MachineState::Alarm},
    {"NetworkProvisioning", MachineState::NetworkProvisioning},
    {"FactoryTest", MachineState::FactoryTest},
    {"FactoryReset", MachineState::FactoryReset},
    {"Shutdown", MachineState::Shutdown},
    {"LeavingDock", MachineState::LeavingDock}
};

static std::unordered_map<std::string, MachineEvent> event_str_to_enum_map = {
    {"ToMapping", MachineEvent::ToMapping},
    {"ToMowing", MachineEvent::ToMowing},
    {"ToRemoteMowing", MachineEvent::ToRemoteMowing},
    {"ToShutdown", MachineEvent::ToShutdown},
    {"ToOtaUpdating", MachineEvent::ToOtaUpdating},
    {"ToCharging", MachineEvent::ToCharging},
    {"ToStandby", MachineEvent::ToStandby},
    {"ToReturningToDock", MachineEvent::ToReturningToDock},
    {"ToAlarm", MachineEvent::ToAlarm},
    {"ToNetworkProvisioning", MachineEvent::ToNetworkProvisioning},
    {"ToFactoryTest", MachineEvent::ToFactoryTest},
    {"ToPaused", MachineEvent::ToPaused},
    {"ToResume", MachineEvent::ToResume},
    {"ToStopped", MachineEvent::ToStopped},
    {"ToFactoryReset", MachineEvent::ToFactoryReset},
    {"ToLeavingDock", MachineEvent::ToLeavingDock}
};

static std::unordered_map<MachineState, MachineEvent> state_to_event_map = {
    {MachineState::StandingBy, MachineEvent::ToStandby},
    {MachineState::OtaUpdating, MachineEvent::ToOtaUpdating},
    {MachineState::Charging, MachineEvent::ToCharging},
    {MachineState::ReturningToDock, MachineEvent::ToReturningToDock},
    {MachineState::Mapping, MachineEvent::ToMapping},
    {MachineState::Mowing, MachineEvent::ToMowing},
    {MachineState::RemoteMowing, MachineEvent::ToRemoteMowing},
    {MachineState::Alarm, MachineEvent::ToAlarm},
    {MachineState::NetworkProvisioning, MachineEvent::ToNetworkProvisioning},
    {MachineState::FactoryTest, MachineEvent::ToFactoryTest},
    {MachineState::FactoryReset, MachineEvent::ToFactoryReset},
    {MachineState::Shutdown, MachineEvent::ToShutdown},
    {MachineState::LeavingDock, MachineEvent::ToLeavingDock}
};

static std::unordered_map<MachineEvent, MachineState> event_to_state_map = {
    {MachineEvent::ToMapping, MachineState::Mapping},
    {MachineEvent::ToMowing, MachineState::Mowing},
    {MachineEvent::ToRemoteMowing, MachineState::RemoteMowing},
    {MachineEvent::ToShutdown, MachineState::Shutdown},
    {MachineEvent::ToOtaUpdating, MachineState::OtaUpdating},
    {MachineEvent::ToCharging, MachineState::Charging},
    {MachineEvent::ToStandby, MachineState::StandingBy},
    {MachineEvent::ToReturningToDock, MachineState::ReturningToDock},
    {MachineEvent::ToAlarm, MachineState::Alarm},
    {MachineEvent::ToNetworkProvisioning, MachineState::NetworkProvisioning},
    {MachineEvent::ToFactoryTest, MachineState::FactoryTest},
    {MachineEvent::ToPaused, MachineState::StandingBy},
    {MachineEvent::ToResume, MachineState::Mowing},
    {MachineEvent::ToStopped, MachineState::StandingBy},
    {MachineEvent::ToFactoryReset, MachineState::FactoryReset},
    {MachineEvent::ToLeavingDock, MachineState::LeavingDock}
};

enum class ReportAppStatue {
  BOOTING = 0,               // 启动中
  STANDBY = 1,               // 待机中
  OTA_UPDATING = 2,          // OTA更新中
  CHARGING = 3,              // 充电中
  BATTERY_FULL = 4,          // 电池已充满
  RETURNING_TO_DOCK = 5,     // 回充中
  MAPPING = 6,               // 建图中
  MOWING = 7,                // 割草中
  REMOTE_MOWING = 8,         // 遥控割草中
  ALARM = 9,                 // 报警中
  NETWORK_PROVISIONING = 10, // 配网中
  FACTORY_RESET = 11,        // 恢复出厂设置中
  SHUTDOWN = 12,             // 关机
  LEAVING_DOCK = 13,         // 退基站中
};

} // namespace machine
