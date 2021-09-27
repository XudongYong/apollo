#include <memory>
#include "cyber/component/component.h"

//namespace apollo{
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class ACCComponent : public Component<> {
 public:
  bool Init() override;
  // bool Proc(const std::shared_ptr<int>& msg0,
  //           const std::shared_ptr<int>& msg1) {return true;};
 private:
  void run();
};
CYBER_REGISTER_COMPONENT(ACCComponent)

//}

