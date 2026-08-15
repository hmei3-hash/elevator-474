# Verification and validation table

Every row starts at NOT TESTED and stays there until the required evidence
exists and has been reviewed. A clean compile is not evidence. An upload
that completes without error is not evidence. Only the artifact named in
the evidence column can move a row off NOT TESTED, and only Hongyi grants
PASS.

| ID and type | Procedure and acceptance criterion | Required evidence | Status |
|---|---|---|---|
| VV-01 unit | Run the host harness: `gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c ../filter.c -lm && ./test_logic`. Accept when every case passes and no case is an empty stub. | Terminal transcript showing the case tally and a zero exit status. | NOT TESTED |
| VV-02 unit | PID output clamping: drive a saturating error and confirm the output stops at the configured clamp. Accept when the returned value equals the clamp exactly. | Host harness output for the clamping case. | NOT TESTED |
| VV-03 unit | Moving average partial window: push fewer samples than the window and confirm the divisor is the sample count. Accept when the average equals the mean of the pushed samples. | Host harness output for the partial-window case. | NOT TESTED |
| VV-04 integration | Stack sizing: run the probe sketch through a full exercise of every task and read the high-water marks. Accept when every task's peak usage is at most half its configured stack. | Serial capture of the probe report table, with the run described. | NOT TESTED |
| VV-05 integration | Queue depth: run the same exercise and read peak occupancy and dropped counts. Accept when every dropped counter is zero and peak is at most half the depth. | Serial capture of the probe queue table. | NOT TESTED |
| VV-06 integration | Protocol copies identical: `diff ElevatorA/link_protocol.h ElevatorB/link_protocol.h`. Accept when the diff is empty. | Terminal transcript of the diff and its exit status. | NOT TESTED |
| VV-07 hardware | Motion driver responds: power the driver, call the init path, and confirm it answers on its control channel. Accept when init reports success on ten consecutive cold boots. | Serial log across ten power cycles, plus a photo of the wiring. | NOT TESTED |
| VV-08 hardware | Ranging sanity: place the sensor at three measured distances and compare reported values against a tape measure. Accept when the error at each point is within the budget recorded here once it is chosen. | Table of commanded versus measured distance, plus a photo of the setup. | NOT TESTED |
| VV-09 hardware | Card reader: present an enrolled card and an unenrolled card. Accept when the enrolled card is reported with a stable UID and the unenrolled card is rejected. | Serial log of both presentations, plus a video of the attempt. | NOT TESTED |
| VV-10 hardware | Display: confirm the panel shows the current mode and floor and updates within its refresh period. Accept when the shown floor matches the physical car position at every stop. | Video of a multi-floor run with the panel in frame. | NOT TESTED |
| VV-11 hardware | Car buttons: press each of the three buttons and confirm exactly one request is registered per press. Accept when no press produces a duplicate or missed request across twenty presses. | Serial log of twenty presses with the registered requests. | NOT TESTED |
| VV-12 system | Heartbeat liveness: with both boards running, confirm Board A never reports a link fault during a five minute idle period. Accept when the fault counter stays at zero. | Serial log covering the full five minutes. | NOT TESTED |
| VV-13 system | Link loss stops the car: with the car moving, cut power to Board B. Accept when the car stops and latches a link fault within the timeout budget recorded here once it is chosen. | Video showing the power cut and the stop, plus the serial log. | NOT TESTED |
| VV-14 system | Fall triggers stop: with the car moving, trigger a fall on Board B. Accept when the car stops and latches a fall fault, and the two faults are distinguishable in the log. | Video of the trigger and stop, plus the serial log showing the fault code. | NOT TESTED |
| VV-15 system | Control loop holds its deadline under load: run the background compute task at full rate and measure control period jitter. Accept when the jitter stays within the budget recorded here once it is chosen. | Captured jitter measurements with the load task confirmed running. | NOT TESTED |
| VV-16 system | Floor accuracy: command each floor ten times from different starting floors and measure the stopping error. Accept when every stop is within the tolerance recorded here once it is chosen. | Table of thirty commanded stops with measured error, plus a photo of the measurement method. | NOT TESTED |

## Status values

- **NOT TESTED** -- no evidence collected yet. Every row starts here.
- **FAIL** -- evidence collected, acceptance criterion not met.
- **PASS** -- evidence collected, criterion met, and Hongyi has signed off.
