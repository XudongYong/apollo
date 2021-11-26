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

#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vc_brake_63.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace acc_fakevhcl {

using ::apollo::drivers::canbus::Byte;

const int32_t Vcbrake63::ID = 0x63;

// public
Vcbrake63::Vcbrake63() { Reset(); }

uint32_t Vcbrake63::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Vcbrake63::UpdateData(uint8_t* data) {
  set_p_vc_brake(data, vc_brake_);
}

void Vcbrake63::Reset() {
  // TODO(All) :  you should check this manually
  vc_brake_ = 0.0;
}

Vcbrake63* Vcbrake63::set_vc_brake(
    double vc_brake) {
  vc_brake_ = vc_brake;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'VC_Brake', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
void Vcbrake63::set_p_vc_brake(uint8_t* data,
    double vc_brake) {
  vc_brake = ProtocolData::BoundedValue(0.0, 1.0, vc_brake);
  int x = vc_brake / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace acc_fakevhcl
}  // namespace canbus
}  // namespace apollo
