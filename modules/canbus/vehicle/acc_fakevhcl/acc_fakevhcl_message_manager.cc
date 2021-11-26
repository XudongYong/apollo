/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/acc_fakevhcl/acc_fakevhcl_message_manager.h"

#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vc_brake_63.h"
#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vc_gas_62.h"

#include "modules/canbus/vehicle/acc_fakevhcl/protocol/cfg_cruise_speed_66.h"
#include "modules/canbus/vehicle/acc_fakevhcl/protocol/dm_brake_61.h"
#include "modules/canbus/vehicle/acc_fakevhcl/protocol/dm_gas_60.h"
#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vhcl_poi_ax_1_65.h"
#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vhcl_velocity_64.h"

namespace apollo {
namespace canbus {
namespace acc_fakevhcl {

Acc_fakevhclMessageManager::Acc_fakevhclMessageManager() {
  // Control Messages
  AddSendProtocolData<Vcbrake63, true>();
  AddSendProtocolData<Vcgas62, true>();

  // Report Messages
  AddRecvProtocolData<Cfgcruisespeed66, true>();
  AddRecvProtocolData<Dmbrake61, true>();
  AddRecvProtocolData<Dmgas60, true>();
  AddRecvProtocolData<Vhclpoiax165, true>();
  AddRecvProtocolData<Vhclvelocity64, true>();
}

Acc_fakevhclMessageManager::~Acc_fakevhclMessageManager() {}

}  // namespace acc_fakevhcl
}  // namespace canbus
}  // namespace apollo
