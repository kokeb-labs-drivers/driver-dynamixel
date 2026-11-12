/**
 * @file quickstart.cpp
 * @brief The Quickstart page's example, compiled by the host build (never run).
 */
#include "my_port.hpp"

// --8<-- [start:quickstart]
#include "dynamixel.hpp"

int move_first_servo(MyPort& port) {
  dynamixel::DYNAMIXEL<MyPort> servos(port); // protocol 2.0, 57 600 bit/s; no I/O yet

  if (auto r = servos.EnsureInitialized(); !r) { // 1. UART up
    return static_cast<int>(r.error);            //    Bus: check the UART and the wiring
  }
  const auto found = servos.Ping(1); // 2. who is there? model, firmware, family
  if (!found) {
    return static_cast<int>(found.error); //    Timeout: wrong ID, baud rate or protocol
  }
  const dynamixel::ServoRef servo{found.value.id, found.value.family};
  if (auto r = servos.SetTorque(servo, true); !r) { // 3. torque on
    return static_cast<int>(r.error);
  }
  return static_cast<int>(servos.SetGoalPosition(servo, 2048).error); // 4. half a turn
}
// --8<-- [end:quickstart]
