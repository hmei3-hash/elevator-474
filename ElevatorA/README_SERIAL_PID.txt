SERIAL PID TUNING BUILD

Default floor setpoints:
  Floor 1 = 30 mm
  Floor 2 = 153 mm

Default travel/target limits:
  lower = 20 mm
  upper = 198 mm

Serial monitor:
  baud = 115200
  line ending = Newline

Commands:
  p <kp> <ki> <kd>
      Set PID live.
      Example:
        p 2.5 0 0

  k
      Show current PID gains.

  g <mm>
      Set an absolute target.
      The target is clamped to the active 20..198 mm envelope.
      Examples:
        g 80
        g 250    -> becomes 198
        g 5      -> becomes 20

  r <delta_mm>
      Relative move from the current target.
      Example:
        r 10
        r -10

  l
      Show current height limits.

  l <min_mm> <max_mm>
      Change the runtime height limits.
      Example:
        l 20 198

  d
      Toggle PID telemetry at about 20 Hz:
        ms,target_mm,pos_mm,error_mm,rate_sps

  s
      Hold current ultrasonic position.

  c
      Clear controller's internal stall latch.

  ?
      Show controller status including PID, limits, target, position and rate.

Safety behavior:
- Every target is clamped to the active limits.
- At/beyond the lower limit, commands that would move farther downward are blocked.
- At/beyond the upper limit, commands that would move farther upward are blocked.
- Motion back toward the legal range is still allowed.
