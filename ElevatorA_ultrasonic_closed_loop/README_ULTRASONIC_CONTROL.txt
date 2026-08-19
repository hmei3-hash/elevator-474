ULTRASONIC CLOSED LOOP - DROP-IN FILES

Replace in ElevatorA/:
  ElevatorA_main.cpp
  control.cpp
  control.h
  cmd.cpp

Add:
  ultrasonic_control_config.h

Keep:
  pid_arduino_bridge.c   (needed when building with Arduino IDE)
  ../logic/pid.c
  ../logic/pid.h

AS5600:
  No longer initialized or used by the controller.
  as5600.cpp/as5600.h may remain in the folder unused, or you may remove them.

IMPORTANT:
  TMC2209 must still initialize successfully. If boot still prints
      stepper (TMC2209) FAIL
  the motor driver remains disabled/not-ready and this control rewrite
  cannot make the motor move.

FIRST SAFE TEST:
  1. Boot with CTRL_ULTRA_KP/KI/KD = 0.
  2. Confirm HC-SR04 reports sensible stable distance.
  3. In Serial Monitor: ?
  4. Set a nearby target, e.g. g <current_mm + 20>.
  5. Set a small P gain, e.g. p 5 0 0.
  6. If the car moves AWAY from the target, immediately stop and change
     CTRL_ULTRA_DIRECTION_SIGN to -1.0f, then rebuild.
  7. Increase Kp gradually. Keep Ki=0 and Kd=0 initially.

FLOOR BUTTONS:
  Measure the HC-SR04 reading at each real floor and fill:
      CTRL_FLOOR0_MM
      CTRL_FLOOR1_MM
      CTRL_FLOOR2_MM
  Until then floor requests are rejected instead of guessing unsafe values.
