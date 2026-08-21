/*
 * pid_arduino_bridge.c
 *
 * Arduino IDE only:
 * Arduino compiles source files in the ElevatorA sketch folder, but the
 * shared PID implementation lives in ../logic/pid.c.
 *
 * PlatformIO already compiles the shared logic source, so exclude this
 * bridge there to avoid duplicate symbol definitions.
 */

#ifndef PLATFORMIO
#include "../logic/pid.c"
#endif
