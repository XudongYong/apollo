#include "modules/ACC_Test/acc_component.h"
#include "modules/ACC_Test/ACC.h"

//namespace apollo{

  bool ACCComponent::Init() {
    AINFO << "ACC component init";
    // start async loop
    apollo::cyber::Async(&ACCComponent::run, this);
    return true;
  }

  void ACCComponent::run(){
    AINFO << "Start ACCTest Proc";
    while (!apollo::cyber::IsShutdown())
    {
      RunACC();
    }  
  }

//}
