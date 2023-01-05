#pragma once

#include <networktables/NetworkTable.h>

#include <functional>

namespace wom {
  class NTBound {
   public:
    NTBound(std::shared_ptr<nt::NetworkTable> table, std::string name, std::shared_ptr<nt::Value> value, std::function<void(std::shared_ptr<nt::Value>)> onUpdateFn)
      : _entry(table->GetEntry(name)), _onUpdate(onUpdateFn) {
        _entry.SetValue(value);
        _listener = _entry.AddListener([=](const nt::EntryNotification &evt) {
          this->_onUpdate(evt.value);
        }, NT_NOTIFY_UPDATE);
      }
    
    ~NTBound() {
      _entry.RemoveListener(_listener);
    }
   protected:
    NT_EntryListener _listener;
    nt::NetworkTableEntry _entry;
    std::function<void(std::shared_ptr<nt::Value>)> _onUpdate;
  };

  class NTBoundDouble : public NTBound {
   public:
    NTBoundDouble(std::shared_ptr<nt::NetworkTable> table, std::string name, double &val)
      : NTBound(table, name, nt::Value::MakeDouble(val), [&val](std::shared_ptr<nt::Value> v) { val = v->GetDouble(); }) {}
  };

  template <typename T>
  class NTBoundUnit : public NTBound {
   public:
    NTBoundUnit(std::shared_ptr<nt::NetworkTable> table, std::string name, units::unit_t<T> &val)
      : NTBound(table, name, nt::Value::MakeDouble(val.value()), [&val](std::shared_ptr<nt::Value> v) { val = units::unit_t<T> { v->GetDouble() }; }) {}
  };
}