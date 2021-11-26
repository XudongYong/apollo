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

#include "modules/canbus/vehicle/poc_vehicle/protocol/acc_info_41.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

using ::apollo::drivers::canbus::Byte;

const int32_t Accinfo41::ID = 0x41;

// public
Accinfo41::Accinfo41() { Reset(); }

uint32_t Accinfo41::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Accinfo41::UpdateData(uint8_t* data) {
  set_p_acc_enabled(data, acc_enabled_);
  set_p_brake(data, brake_);
  set_p_throttle(data, throttle_);
}

void Accinfo41::Reset() {
  // TODO(All) :  you should check this manually
  acc_enabled_ = false;
  brake_ = 0;
  throttle_ = 0;
}

Accinfo41* Accinfo41::set_acc_enabled(
    bool acc_enabled) {
  acc_enabled_ = acc_enabled;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'ACC_Enabled', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Accinfo41::set_p_acc_enabled(uint8_t* data,
    bool acc_enabled) {
  int x = acc_enabled;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 1);
}


Accinfo41* Accinfo41::set_brake(
    int brake) {
  brake_ = brake;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
void Accinfo41::set_p_brake(uint8_t* data,
    int brake) {
  brake = ProtocolData::BoundedValue(0, 100, brake);
  int x = brake;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Accinfo41* Accinfo41::set_throttle(
    int throttle) {
  throttle_ = throttle;
  return this;
 }

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
void Accinfo41::set_p_throttle(uint8_t* data,
    int throttle) {
  throttle = ProtocolData::BoundedValue(0, 100, throttle);
  int x = throttle;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo
