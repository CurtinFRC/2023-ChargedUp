#pragma once

#include <frc/XboxController.h>
#include <frc/PS4Controller.h>

#include "frc/GenericHID.h"
namespace wom {
class Controller {
 public:
  Controller(int port) : _port(port) {};

  virtual double GetRightX();
  virtual double GetRightY();
  virtual double GetLeftX();
  virtual double GetLeftY();
  virtual double GetLeftTrigger();
  virtual double GetRightTrigger();
  virtual bool GetLeftBumperPressed();
  virtual bool GetRightBumperPressed();
  virtual bool GetLeftJoystickPressed();
  virtual bool GetRightJoystickPressed();
  virtual bool GetCPAD_BottomPressed();
  virtual bool GetCPAD_RightPressed();
  virtual bool GetCPAD_LeftPressed();
  virtual bool GetCPAD_TopPressed();
  virtual bool GetMiniLeftButtonPressed();
  virtual bool GetMiniRightButtonPressed();
  virtual bool GetLogoButtonPressed();

  virtual frc::BooleanEvent LeftBumperPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent RightBumperPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent LeftJoystickPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent RightJoystickPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent CPAD_BottomPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent CPAD_RightPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent CPAD_LeftPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent CPAD_TopPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent MiniLeftButtonPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent MiniRightButtonPressed(frc::EventLoop* loop);
  virtual frc::BooleanEvent LogoButtonPressed(frc::EventLoop* loop);

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

  virtual frc::BooleanEvent LeftBumperPressed(frc::EventLoop* loop) override {   return xbox->LeftBumper(loop);   }
  virtual frc::BooleanEvent RightBumperPressed(frc::EventLoop* loop) override {   return xbox->RightBumper(loop);   }
  virtual frc::BooleanEvent LeftJoystickPressed(frc::EventLoop* loop) override {   return xbox->LeftStick(loop);   }
  virtual frc::BooleanEvent RightJoystickPressed(frc::EventLoop* loop) override {   return xbox->RightStick(loop);   }
  virtual frc::BooleanEvent CPAD_BottomPressed(frc::EventLoop* loop) override {   return xbox->A(loop);   }
  virtual frc::BooleanEvent CPAD_RightPressed(frc::EventLoop* loop) override {   return xbox->B(loop);   }
  virtual frc::BooleanEvent CPAD_LeftPressed(frc::EventLoop* loop) override {   return xbox->X(loop);   }
  virtual frc::BooleanEvent CPAD_TopPressed(frc::EventLoop* loop) override {   return xbox->Y(loop);   }
  virtual frc::BooleanEvent MiniLeftButtonPressed(frc::EventLoop* loop) override {   return   xbox->Back(loop);   }
  virtual frc::BooleanEvent MiniRightButtonPressed(frc::EventLoop* loop) override {   return   xbox->Start(loop);   }

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

  virtual frc::BooleanEvent LeftBumperPressed(frc::EventLoop *loop) override {   return ps4->L1(loop);   }
  virtual frc::BooleanEvent RightBumperPressed(frc::EventLoop *loop) override {   return ps4->R1(loop);   }
  virtual frc::BooleanEvent LeftJoystickPressed(frc::EventLoop *loop) override {   return ps4->L3(loop);   }
  virtual frc::BooleanEvent RightJoystickPressed(frc::EventLoop *loop) override {   return ps4->R3(loop);   }
  virtual frc::BooleanEvent CPAD_BottomPressed(frc::EventLoop *loop) override {   return ps4->Cross(loop);   }
  virtual frc::BooleanEvent CPAD_RightPressed(frc::EventLoop *loop) override {   return ps4->Circle(loop);   }
  virtual frc::BooleanEvent CPAD_LeftPressed(frc::EventLoop *loop) override {   return ps4->Square(loop);   }
  virtual frc::BooleanEvent CPAD_TopPressed(frc::EventLoop *loop) override {   return ps4->Triangle(loop);   }
  virtual frc::BooleanEvent MiniLeftButtonPressed(frc::EventLoop *loop) override {   return   ps4->Share(loop);   }
  virtual frc::BooleanEvent MiniRightButtonPressed(frc::EventLoop *loop) override {   return   ps4->Options(loop);   }

 private:
  frc::PS4Controller *ps4;
};
} // wom