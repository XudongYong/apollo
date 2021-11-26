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

#include "modules/canbus/vehicle/poc_vehicle/poc_vehicle_message_manager.h"

#include "modules/canbus/vehicle/poc_vehicle/protocol/acc_info_41.h"

#include "modules/canbus/vehicle/poc_vehicle/protocol/dash_commands_42.h"
#include "modules/canbus/vehicle/poc_vehicle/protocol/pt_info_43.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

Poc_vehicleMessageManager::Poc_vehicleMessageManager() {
  // Control Messages
  AddSendProtocolData<Accinfo41, true>();

  // Report Messages
  AddRecvProtocolData<Dashcommands42, true>();
  AddRecvProtocolData<Ptinfo43, true>();
}

Poc_vehicleMessageManager::~Poc_vehicleMessageManager() {}

}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo
