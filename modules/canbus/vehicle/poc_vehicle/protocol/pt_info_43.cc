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

#include "modules/canbus/vehicle/poc_vehicle/protocol/pt_info_43.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

using ::apollo::drivers::canbus::Byte;

Ptinfo43::Ptinfo43() {}
const int32_t Ptinfo43::ID = 0x43;

void Ptinfo43::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_poc_vehicle()->mutable_pt_info_43()->set_lateral_acceleration(lateral_acceleration(bytes, length));
  chassis->mutable_poc_vehicle()->mutable_pt_info_43()->set_vehicle_speed(vehicle_speed(bytes, length));
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 12, 'name': 'lateral_acceleration', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ptinfo43::lateral_acceleration(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 12, 'name': 'vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|300]', 'physical_unit': 'mph', 'precision': 1.0, 'type': 'int'}
int Ptinfo43::vehicle_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo
