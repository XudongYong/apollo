#include "cyber/time/clock.h"
#include "modules/ACC_Test/acc_component.h"
#include "modules/ACC_Test/ACC.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "cyber/time/time.h"

using ::apollo::cyber::Clock;
using ::apollo::control::ControlCommand;

namespace apollo{

  bool ACCComponent::Init() {
    AINFO << "ACC component init";
   // InitACC();
   control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    return true;
  }

  bool ACCComponent::Proc(const std::shared_ptr<ContiRadar>& radar, const std::shared_ptr<canbus::Chassis>& chassis) {
    AINFO << "Enter ACC module, message timestamp: "
        << radar->header().timestamp_sec() << " current timestamp "
        << Clock::NowInSeconds();
      if(chassis->has_speed_mps())
      {
        vehicleSpeed = chassis->speed_mps();
        AINFO << "speed: " << vehicleSpeed;
      }
      if(chassis->has_acceleration())
      {
        
        vehicleXAcceleration = chassis->acceleration() -30;
        AINFO << "XAcceleration: " << vehicleXAcceleration ;
      }
    
    this->accEnabled = chassis->driving_mode() == canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE;

    Run(radar);
    return true;
  }

  void ACCComponent::Run(const std::shared_ptr<ContiRadar>& radar){
    AINFO << "Start calling ACC ECU controller";
    ContiRadar raw_obstacles = *radar;
    // format the contiobs into double arrays needed by the c algorithm
    double objValues[raw_obstacles.contiobs_size() * 4];
    int index = 0;
    for (const auto& contiobs : raw_obstacles.contiobs()) {
      double longitude_dist = contiobs.longitude_dist();
      double lateral_dist = contiobs.lateral_dist();
      double longitude_vel = contiobs.longitude_vel();
      double rcs = contiobs.rcs();
      objValues[index++] = longitude_dist;
      objValues[index++] = lateral_dist;
      objValues[index++] = longitude_vel;
      objValues[index++] = rcs;
      AINFO << "longitude_dist: " << longitude_dist << " lateral_dist: " << lateral_dist << " longitude_vel: " << longitude_vel << " rcs: " << rcs;
  }
    AccCommand result = RunACC(objValues, vehicleSpeed, vehicleXAcceleration, accEnabled);
    AINFO << "break: " << result.breakpedal;
    AINFO << "gas: "  << result.throttle;
    AINFO << "#################find target: " << result.findtarget;
    AINFO << "###1 acc enabled: " << accEnabled;
    ControlCommand control_command;
    control_command.mutable_header()->set_timestamp_sec(::apollo::cyber::Time::Now().ToSecond());
    control_command.set_brake(result.breakpedal);
    control_command.set_throttle(result.throttle);
    control_command.set_driving_mode(
      this->accEnabled 
        ? canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE 
        : canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);

    control_cmd_writer_->Write(control_command);
  }

  ACCComponent::~ACCComponent()
  {
   // ShutDownACC();
  }

}
