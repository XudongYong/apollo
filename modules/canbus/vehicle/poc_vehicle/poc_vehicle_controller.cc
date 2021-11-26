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

#include "modules/canbus/vehicle/poc_vehicle/poc_vehicle_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/poc_vehicle/poc_vehicle_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "cyber/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace poc_vehicle {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode Poc_vehicleController::Init(
	const VehicleParameter& params,
	CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "Poc_vehicleController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  acc_info_41_ = dynamic_cast<Accinfo41*>
          (message_manager_->GetMutableProtocolDataById(Accinfo41::ID));
  if (acc_info_41_ == nullptr) {
     AERROR << "Accinfo41 does not exist in the Poc_vehicleMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Accinfo41::ID, acc_info_41_, false);

  if (params.driving_mode() == Chassis::COMPLETE_AUTO_DRIVE)
  {
    // Initialize the PT.acc_enabled to be true 
    this->EnableAutoMode();
  }

  // need sleep to ensure all messages received
  AINFO << "Poc_vehicleController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

Poc_vehicleController::~Poc_vehicleController() {}

bool Poc_vehicleController::Start() {
  if (!is_initialized_) {
    AERROR << "Poc_vehicleController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void Poc_vehicleController::Stop() {
  if (!is_initialized_) {
    AERROR << "Poc_vehicleController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "Poc_vehicleController stopped.";
  }
}

Chassis Poc_vehicleController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // POC vehicle chassis logic

  // check if there is no poc_vehicle, no chassis detail can be retrieved and return
  if (!chassis_detail.has_poc_vehicle()) {
    AERROR << "NO poc_vehicle chassis information!";
    return chassis_;
  }

  Poc_vehicle poc_vehicle = chassis_detail.poc_vehicle();
  
  // 5
  if (poc_vehicle.has_pt_info_43() &&
      poc_vehicle.pt_info_43().has_vehicle_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(poc_vehicle.pt_info_43().vehicle_speed()));
    }
  else {
    chassis_.set_speed_mps(0);
  }

  // 8
  if (poc_vehicle.has_acc_info_41() && poc_vehicle.acc_info_41().has_throttle()) {
    chassis_.set_throttle_percentage(
        static_cast<float>(poc_vehicle.acc_info_41().throttle()));
  } else {
    chassis_.set_throttle_percentage(0);
  }

  // 9
  AINFO << "###1 has_acc_info_41: " << poc_vehicle.has_acc_info_41() 
        << " has_brake: " << poc_vehicle.acc_info_41().has_brake();
  if (poc_vehicle.has_acc_info_41() && poc_vehicle.acc_info_41().has_brake()) {
    chassis_.set_brake_percentage(
        static_cast<float>(poc_vehicle.acc_info_41().brake()));
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 35
  if (poc_vehicle.has_pt_info_43() && poc_vehicle.pt_info_43().has_lateral_acceleration()) {
    chassis_.set_acceleration(
        static_cast<float>(poc_vehicle.pt_info_43().lateral_acceleration()));
  } else {
    chassis_.set_acceleration(0);
  }

  // Todo: add dash ecu command here...
  AINFO << "###1 has_dash_commands_42: " << poc_vehicle.has_dash_commands_42() 
        << " has_acc_enabled: " << poc_vehicle.dash_commands_42().has_acc_enabled()
        << " acc_info_41_->acc_enabled: " << acc_info_41_->acc_enabled();
  if (poc_vehicle.has_dash_commands_42() && poc_vehicle.dash_commands_42().has_acc_enabled()) {
    bool accEnabled = poc_vehicle.dash_commands_42().acc_enabled();
    AINFO << "###1 accEnabled: " << accEnabled;
    if (accEnabled)
    {
      chassis_.set_driving_mode(Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
    }
    else
    {
      chassis_.set_driving_mode(Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);      
    }
  }

  return chassis_;
}

void Poc_vehicleController::Emergency() {
    set_driving_mode(Chassis::EMERGENCY_MODE);
    ResetProtocol();
}

ErrorCode Poc_vehicleController::EnableAutoMode() {
  acc_info_41_->set_acc_enabled(true);
  AINFO << "###1 Switch to COMPLETE_AUTO_DRIVE ok.";

  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  return ErrorCode::OK;
}

ErrorCode Poc_vehicleController::DisableAutoMode() {
  ResetProtocol();
  acc_info_41_->set_acc_enabled(false);
  //can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "###1 Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode Poc_vehicleController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
}

ErrorCode Poc_vehicleController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void Poc_vehicleController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void Poc_vehicleController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  AINFO << "vehiclesetbrake: "  << pedal;
  acc_info_41_->set_brake(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void Poc_vehicleController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  acc_info_41_->set_throttle(pedal);
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void Poc_vehicleController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
}

// poc_vehicle default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void Poc_vehicleController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support Steer yet.";
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void Poc_vehicleController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support SetEpbBreak yet.";
}

void Poc_vehicleController::SetEpbBreak(const ControlCommand& command) {
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support SetEpbBreak yet.";
}

void Poc_vehicleController::SetBeam(const ControlCommand& command) {
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support SetEpbBreak yet.";
}

void Poc_vehicleController::SetHorn(const ControlCommand& command) {
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support SetEpbBreak yet.";
}

void Poc_vehicleController::SetTurningSignal(const ControlCommand& command) {
  /* Not supported yet*/
  AINFO << "Poc_vehicle not support SetEpbBreak yet.";
}

void Poc_vehicleController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool Poc_vehicleController::CheckChassisError() {
  /* No check for now as we use simulator, assume no error
  */
  return false;
}

void Poc_vehicleController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::cyber::Time::Now().ToMicrosecond();
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    // if (CheckChassisError()) {
    //   set_chassis_error_code(Chassis::CHASSIS_ERROR);
    //   emergency_mode = true;
    // }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in Poc_vehicleController looping process:"
          << elapsed.count();
    }
  }
}

bool Poc_vehicleController::CheckResponse(const int32_t flags, bool need_wait) {
 /* No check for now as we use simulator, assume no failure
  */
  return true;
}

void Poc_vehicleController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t Poc_vehicleController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode Poc_vehicleController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void Poc_vehicleController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace poc_vehicle
}  // namespace canbus
}  // namespace apollo
