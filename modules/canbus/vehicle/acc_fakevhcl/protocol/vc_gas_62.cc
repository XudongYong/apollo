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

#include "modules/canbus/vehicle/acc_fakevhcl/protocol/vc_gas_62.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace acc_fakevhcl {

using ::apollo::drivers::canbus::Byte;

const int32_t Vcgas62::ID = 0x62;

// public
Vcgas62::Vcgas62() { Reset(); }

uint32_t Vcgas62::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Vcgas62::UpdateData(uint8_t* data) {
  set_p_vc_gas(data, vc_gas_);
}

void Vcgas62::Reset() {
  // TODO(All) :  you should check this manually
  vc_gas_ = 0.0;
}

Vcgas62* Vcgas62::set_vc_gas(
    double vc_gas) {
  vc_gas_ = vc_gas;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'VC_Gas', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
void Vcgas62::set_p_vc_gas(uint8_t* data,
    double vc_gas) {
  vc_gas = ProtocolData::BoundedValue(0.0, 1.0, vc_gas);
  int x = vc_gas / 0.100000;
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
