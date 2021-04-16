#ifndef LEO_FIRMWARE_INCLUDE_WHEEL_CONTROLLER_H_
#define LEO_FIRMWARE_INCLUDE_WHEEL_CONTROLLER_H_

#include <cstddef>
#include <cstdint>
#include <utility>

#include <hMotor.h>
#include <hPIDRegulator.h>

#include <leo_firmware/utils.h>

class WheelController {
 public:
  WheelController(hFramework::hMotor& motor, const bool polarity);
  void update(uint32_t dt);
  void setSpeed(float speed);
  float getSpeed();
  int16_t getPower();

  int32_t getDistance();
  void resetDistance();

  void reset();
  void turnOff();
  void turnOn();

 private:
  hFramework::hMotor& motor_;
  hFramework::hPIDRegulator v_reg_;
  CircularBuffer<std::pair<int32_t, uint32_t>> encoder_buffer_;

  int16_t power_;

  bool turned_on_;

  int32_t ticks_now_;
  int32_t ticks_offset_;
  int32_t ticks_sum_;
  uint32_t dt_sum_;

  float v_target_;
  float v_now_;

  const bool polarity_;

  static constexpr int ENCODER_BUFFER_SIZE = 10;
  static constexpr float V_RANGE = 1000.0;
  static constexpr float WHEEL_VELOCITY_REJECTION_THRESHOLD = 2.0;
};

#endif  // LEO_FIRMWARE_INCLUDE_WHEEL_CONTROLLER_H_
