#include "cyber/time/clock.h"
#include "modules/ACC_Test/acc_component.h"
#include "modules/ACC_Test/ACC.h"

using ::apollo::cyber::Clock;

namespace apollo{

  bool ACCComponent::Init() {
    AINFO << "ACC component init";
    InitACC();
    return true;
  }

  bool ACCComponent::Proc(const std::shared_ptr<ContiRadar>& message) {
    AINFO << "Enter ACC module, message timestamp: "
        << message->header().timestamp_sec() << " current timestamp "
        << Clock::NowInSeconds();
    Run(message);
    return true;
  }

  void ACCComponent::Run(const std::shared_ptr<ContiRadar>& message){
    AINFO << "Start calling ACC ECU controller";
    ContiRadar raw_obstacles = *message;
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
    RunACC(objValues); 
  }

  ACCComponent::~ACCComponent()
  {
    ShutDownACC();
  }

}
