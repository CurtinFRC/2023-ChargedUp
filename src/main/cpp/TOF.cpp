#include "TOF.h"

TOF::TOF (frc::I2C::Port port):_I2C(port, 0x29), _notifier([this]() { this->UpdateNow(); }) {
  //starts continuous measurment
  uint8_t data[3] = { 0x00, 0x87, 0x21};
  _I2C.WriteBulk(data, 3);

  //runs UpdateNow every 20 milliseconds
  _notifier.StartPeriodic(20_ms);
}

//returns the current value
std::optional<units::meter_t> TOF::GetDistance() {
  return _currentValue;
}

void TOF::UpdateNow() {
  //clear interupt and starts new measurement
  uint8_t send[3] = {0x00, 0x86, 0x01};
  _I2C.WriteBulk(send, 3);

  //reads status
  uint8_t status;
  send[0] = 0x00;
  send[1] = 0x89;
  _I2C.Transaction(send, 2, &status, 1);

  //reads the current distance in millimeters
  uint8_t distance_mm_buf[2];
  send[0] = 0x00;
  send[1] = 0x96;
  _I2C.Transaction(send, 2, distance_mm_buf, 2);
  uint16_t distance_mm = (distance_mm_buf[0] << 8) | distance_mm_buf[1];

  //check if the status is correct
  // if (status == 9) //@Darcey ERROR 
  //   _currentValue = units::millimeter_t{distance_mm};
  // else
  //   _currentValue = {};
}