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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

class Accinfo41 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Accinfo41();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'ACC_Enabled', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Accinfo41* set_acc_enabled(bool acc_enabled);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  Accinfo41* set_brake(int brake);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  Accinfo41* set_throttle(int throttle);

  bool acc_enabled() { return acc_enabled_; }

 private:

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'ACC_Enabled', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_acc_enabled(uint8_t* data, bool acc_enabled);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  void set_p_brake(uint8_t* data, int brake);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  void set_p_throttle(uint8_t* data, int throttle);

 private:
  bool acc_enabled_;
  int brake_;
  int throttle_;
};

}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo


