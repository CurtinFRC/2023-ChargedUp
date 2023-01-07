#include "Gyro.h"

using namespace wom;

#ifdef PLATFORM_ROBORIO
  #include <thread>
  #include "AHRS.h"
  class NavX::Impl {
   public:
    Impl() : ahrs(frc::SPI::kMXP) { }

    void Calibrate() {
      ahrs.Calibrate();
    }

    void Reset() {
      ahrs.Reset();
    }

    double GetAngle() const {
      return ahrs.GetAngle();
    }

    double GetRate() const {
      return ahrs.GetRate();
    }

    void SetAngle(units::radian_t offset) {
      Reset();
      this->offset = offset.convert<units::degree>().value();
    }
   private:
    AHRS ahrs;
    double offset;
  };
#else
  class NavX::Impl {
   public:
    void Calibrate() {}
    void Reset() { angle = 0; }
    double GetAngle() const { return angle; }
    // TODO:
    double GetRate() const { return 0; }

    void SetAngle(units::radian_t offset) {
      angle = offset.convert<units::degree>().value();
    }
   private:
    double angle{0};
  };
#endif

class NavXSimGyro : public sim::SimCapableGyro {
 public:
  NavXSimGyro(NavX *navx) : navx(navx) {}
  void SetAngle(units::radian_t angle) override {
    navx->SetAngle(angle);
  }
 private:
  NavX *navx;
};

NavX::NavX() : impl(new NavX::Impl()) { }
NavX::~NavX() {
  delete impl;
}

void NavX::Calibrate() {
  impl->Calibrate();
}

void NavX::Reset() {
  impl->Reset();
}

double NavX::GetAngle() const {
  return impl->GetAngle();
}

double NavX::GetRate() const {
  return impl->GetRate();
}

void NavX::SetAngle(units::radian_t angle) {
  impl->SetAngle(angle);
}

std::shared_ptr<sim::SimCapableGyro> NavX::MakeSimGyro() {
  return std::make_shared<NavXSimGyro>(this);
}
