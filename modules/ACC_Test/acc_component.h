#include <memory>
#include "cyber/component/component.h"
#include "modules/drivers/proto/conti_radar.pb.h"

namespace apollo{
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::drivers::ContiRadar;

class ACCComponent : public Component<ContiRadar> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ContiRadar>& message) override;
  ~ACCComponent();

 private:
  void Run(const std::shared_ptr<ContiRadar>& message);
};
CYBER_REGISTER_COMPONENT(ACCComponent)

}

