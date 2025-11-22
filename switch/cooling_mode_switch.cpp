
#include "esphome/core/log.h"
#include "./cooling_mode_switch.h"

namespace esphome {
namespace toshiba_uart {

void CoolingModeSwitch::write_state(bool state) {
  this->parent_->set_cooling_mode(state);
}

}  // namespace toshiba_uart
}  // namespace esphome
