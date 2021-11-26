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

#include "modules/canbus/vehicle/poc_vehicle/protocol/dash_commands_42.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

using ::apollo::drivers::canbus::Byte;

Dashcommands42::Dashcommands42() {}
const int32_t Dashcommands42::ID = 0x42;

void Dashcommands42::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_poc_vehicle()->mutable_dash_commands_42()->set_acc_enabled(acc_enabled(bytes, length));
  chassis->mutable_poc_vehicle()->mutable_dash_commands_42()->set_minimum_follow_distance(minimum_follow_distance(bytes, length));
  chassis->mutable_poc_vehicle()->mutable_dash_commands_42()->set_target_vehicle_speed(target_vehicle_speed(bytes, length));
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'acc_enabled', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Dashcommands42::acc_enabled(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'minimum_follow_distance', 'offset': 0.0, 'order': 'intel', 'physical_range': '[10|150]', 'physical_unit': 'ft', 'precision': 1.0, 'type': 'int'}
int Dashcommands42::minimum_follow_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'target_vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|200]', 'physical_unit': 'mph', 'precision': 1.0, 'type': 'int'}
int Dashcommands42::target_vehicle_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo
