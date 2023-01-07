#pragma once

#include <networktables/NetworkTable.h>

#include <functional>

namespace wom {
  class NTBound {
   public:
    NTBound(std::shared_ptr<nt::NetworkTable> table, std::string name, const nt::Value &value, std::function<void(const nt::Value &)> onUpdateFn)
      : _table(table), _entry(table->GetEntry(name)), _onUpdate(onUpdateFn) {
        _entry.SetValue(value);
        // _listener = table->AddListener(name, , ([=](const nt::EntryNotification &evt) {
        //   this->_onUpdate(evt.value);
        // }, NT_NOTIFY_UPDATE);
        _listener = table->AddListener(name, nt::EventFlags::kValueAll, ([this](nt::NetworkTable *table, std::string_view key, const nt::Event &event) {
          this->_onUpdate(event.GetValueEventData()->value);
        }));
      }
    
    ~NTBound() {
      _table->RemoveListener(_listener);
    }
   protected:
    NT_Listener _listener;
    std::shared_ptr<nt::NetworkTable> _table;
    nt::NetworkTableEntry _entry;
    std::function<void(const nt::Value &)> _onUpdate;
  };

  class NTBoundDouble : public NTBound {
   public:
    NTBoundDouble(std::shared_ptr<nt::NetworkTable> table, std::string name, double &val)
      : NTBound(table, name, nt::Value::MakeDouble(val), [&val](const nt::Value &v) { val = v.GetDouble(); }) {}
  };

  template <typename T>
  class NTBoundUnit : public NTBound {
   public:
    NTBoundUnit(std::shared_ptr<nt::NetworkTable> table, std::string name, units::unit_t<T> &val)
      : NTBound(table, name, nt::Value::MakeDouble(val.value()), [&val](const nt::Value &v) { val = units::unit_t<T> { v.GetDouble() }; }) {}
  };
}