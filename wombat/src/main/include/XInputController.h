#pragma once

#include <frc/XboxController.h>
#include <frc/PS4Controller.h>

namespace wom {
class Controller {
 public:
  Controller(int port) : _port(port) {};

  virtual double GetRightX() {   return false;   };
  virtual double GetRightY() {   return false;   };
  virtual double GetLeftX() {   return false;   };
  virtual double GetLeftY() {   return false;   };
  virtual double GetLeftTrigger() {   return false;   };
  virtual double GetRightTrigger() {   return false;   };
  virtual bool GetLeftBumperPressed() {   return false;   };
  virtual bool GetRightBumperPressed() {   return false;   };
  virtual bool GetLeftJoystickPressed() {   return false;   };
  virtual bool GetRightJoystickPressed() {   return false;   };
  virtual bool GetCPAD_BottomPressed() {   return false;   };
  virtual bool GetCPAD_RightPressed() {   return false;   };
  virtual bool GetCPAD_LeftPressed() {   return false;   };
  virtual bool GetCPAD_TopPressed() {   return false;   };
  virtual bool GetMiniLeftButtonPressed() {   return false;   };
  virtual bool GetMiniRightButtonPressed() {   return false;   };
  virtual bool GetLogoButtonPressed() {   return false;   };

 private:
  int _port;
};

class XboxController : public Controller {
 public:
  XboxController(int port) : Controller(port) {
    xbox = new frc::XboxController(port);
  };

  virtual double GetRightX() override {   return xbox->GetRightX();   }
  virtual double GetRightY() override {   return xbox->GetRightY();   }
  virtual double GetLeftX() override {   return xbox->GetLeftX();   }
  virtual double GetLeftY() override {   return xbox->GetLeftY();   }
  virtual double GetLeftTrigger() override {   return xbox->GetLeftTriggerAxis();   }
  virtual double GetRightTrigger() override {   return xbox->GetRightTriggerAxis();   }
  virtual bool GetLeftBumperPressed() override {   return xbox->GetLeftBumper();   }
  virtual bool GetRightBumperPressed() override {   return xbox->GetRightBumper();   }
  virtual bool GetLeftJoystickPressed() override {   return xbox->GetLeftStickButton();   }
  virtual bool GetRightJoystickPressed() override {   return xbox->GetRightStickButton();   }
  virtual bool GetCPAD_BottomPressed() override {   return xbox->GetAButton();   }
  virtual bool GetCPAD_RightPressed() override {   return xbox->GetBButton();   }
  virtual bool GetCPAD_LeftPressed() override {   return xbox->GetXButton();   }
  virtual bool GetCPAD_TopPressed() override {   return xbox->GetYButton();   }
  virtual bool GetMiniLeftButtonPressed() override {   return   xbox->GetBackButton();   }
  virtual bool GetMiniRightButtonPressed() override {   return   xbox->GetStartButton();   }
  /**
   * @brief This function can not be used for Xbox controllers as off right now
   * @returns false, always
  */
  virtual bool GetLogoButtonPressed() override {   return false;   }

 private:
  frc::XboxController *xbox;
};



class PS4Controller : public Controller {
 public:
  PS4Controller(int port) : Controller(port) {
    ps4 = new frc::PS4Controller(port);
  };

  virtual double GetRightX() override {   return ps4->GetRightX();   }
  virtual double GetRightY() override {   return ps4->GetRightY();   }
  virtual double GetLeftX() override {   return ps4->GetLeftX();   }
  virtual double GetLeftY() override {   return ps4->GetLeftY();   }
  virtual double GetLeftTrigger() override {   return ps4->GetL2Button();   }
  virtual double GetRightTrigger() override {   return ps4->GetR2Button();   }
  virtual bool GetLeftBumperPressed() override {   return ps4->GetL1Button();   }
  virtual bool GetRightBumperPressed() override {   return ps4->GetR1Button();   }
  virtual bool GetLeftJoystickPressed() override {   return ps4->GetL3Button();   }
  virtual bool GetRightJoystickPressed() override {   return ps4->GetR3Button();   }
  virtual bool GetCPAD_BottomPressed() override {   return ps4->GetCrossButton();   }
  virtual bool GetCPAD_RightPressed() override {   return ps4->GetCircleButton();   }
  virtual bool GetCPAD_LeftPressed() override {   return ps4->GetSquareButton();   }
  virtual bool GetCPAD_TopPressed() override {   return ps4->GetTriangleButton();   }
  virtual bool GetMiniLeftButtonPressed() override {   return   ps4->GetShareButton();   }
  virtual bool GetMiniRightButtonPressed() override {   return   ps4->GetOptionsButton();   }
  virtual bool GetLogoButtonPressed() override {   return ps4->GetPSButton();   }

 private:
  frc::PS4Controller *ps4;
};
} // wom