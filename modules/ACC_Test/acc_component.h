#include <memory>
#include "cyber/component/component.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo{
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::drivers::ContiRadar;
using apollo::control::ControlCommand;

class ACCComponent : public Component<ContiRadar, canbus::Chassis> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ContiRadar>& radar, const std::shared_ptr<canbus::Chassis>& chassis) override;
  ~ACCComponent();

 private:
  void Run(const std::shared_ptr<ContiRadar>& radar);
  double vehicleSpeed;
  double vehicleXAcceleration;
  bool accEnabled;
  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
};
CYBER_REGISTER_COMPONENT(ACCComponent)

}

