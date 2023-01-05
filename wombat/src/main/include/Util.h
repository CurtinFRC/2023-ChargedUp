#pragma once

namespace wom {
  template<typename T>
  T&& invert(T &&system) {
    system.SetInverted(true);
    return system;
  }
}