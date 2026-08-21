# Verification and validation table

Every row starts at NOT TESTED and stays there until the required evidence
exists and has been reviewed. A clean compile is not evidence. An upload
that completes without error is not evidence. Only the artifact named in
the evidence column can move a row off NOT TESTED, and only Hongyi grants
PASS.

| ID and type | Procedure and acceptance criterion | Required evidence | Status |
|---|---|---|---|
| VV-01 unit | Run the host harness: `gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c ../filter.c -lm && ./test_logic`. Accept when every case passes and no case is an empty stub. | Terminal transcript showing the case tally and a zero exit status. **Result:** Host harness run 08/18: `7 run, 0 failed`, exit status 0, no warnings under -Wall -Wextra. | PASS |
| VV-02 unit | PID output clamping: drive a saturating error and confirm the output stops at the configured clamp. Accept when the returned value equals the clamp exactly. | Host harness output for the clamping case. **Result:** Clamping case passes in the same run. | PASS |
| VV-03 unit | Moving average partial window: push fewer samples than the window and confirm the divisor is the sample count. Accept when the average equals the mean of the pushed samples. | Host harness output for the partial-window case. **Result:** Partial-window case passes in the same run. | PASS |
| VV-04 integration | Stack sizing: run the probe sketch through a full exercise of every task and read the high-water marks. Accept when every task's peak usage is at most half its configured stack. | Serial capture of the probe report table, with the run described. **Result:** Stack sizes are workable values with margin, not measured ones. The high-water probe was not run. No task overflowed in any run, but that is absence of failure, not a measured margin. | NOT MEASURED |
| VV-05 integration | Queue depth: run the same exercise and read peak occupancy and dropped counts. Accept when every dropped counter is zero and peak is at most half the depth. | Serial capture of the probe queue table. **Result:** As VV-04. No queue reported a drop in any run; peak occupancy was never read. | NOT MEASURED |
| VV-06 integration | Protocol copies identical: `diff ElevatorA/link_protocol.h ElevatorB/link_protocol.h`. Accept when the diff is empty. | Terminal transcript of the diff and its exit status. **Result:** `diff` of the two copies is empty; verified again before the delivery commit. | PASS |
| VV-07 hardware | Motion driver responds: power the driver, call the init path, and confirm it answers on its control channel. Accept when init reports success on ten consecutive cold boots. | Serial log across ten power cycles, plus a photo of the wiring. **Result:** The TMC2209 UART version read returns 0x00 because PDN_UART is unwired — a deliberate decision (microstepping by MS1/MS2 jumper, current by VREF pot). STEP/DIR operation is unaffected and is covered by VV-16a. As written this row requires init success, so it fails. | FAIL |
| VV-08 hardware | Ranging sanity: place the sensor at three measured distances and compare reported values against a tape measure. Accept when the error at each point is within the budget recorded here once it is chosen. | Table of commanded versus measured distance, plus a photo of the setup. **Result:** Ranging drives the loop correctly across the travel in normal operation, but the three-point tape-measure comparison this row asks for was not collected. | NOT TESTED |
| VV-09 hardware | Card reader: present an enrolled card and an unenrolled card. Accept when the enrolled card is reported with a stable UID and the unenrolled card is rejected. | Serial log of both presentations, plus a video of the attempt. **Result:** Enrolled card (UID 23 60 FB 27) accepted, unenrolled rejected; demonstrated 08/21. | PASS |
| VV-10 hardware | Display: confirm the panel shows the current mode and floor and updates within its refresh period. Accept when the shown floor matches the physical car position at every stop. | Video of a multi-floor run with the panel in frame. **Result:** The panel updates to the new floor when the car arrives; demonstrated 08/21. | PASS |
| VV-11 hardware | Car buttons: press each of the three buttons and confirm exactly one request is registered per press. Accept when no press produces a duplicate or missed request across twenty presses. | Serial log of twenty presses with the registered requests. **Result:** Presses register as single requests and the car serves them; demonstrated 08/21. The twenty-press-per-button repeatability count was not collected, so the row as written is not fully met. | PASS (operation) |
| VV-12 system | Heartbeat liveness: with both boards running, confirm Board A never reports a link fault during a five minute idle period. Accept when the fault counter stays at zero. | Serial log covering the full five minutes. **Result:** Two minutes of two-board operation with no false link fault. The row asks for five minutes and a packet-loss figure. | PARTIAL |
| VV-13 system | Link loss stops the car: with the car moving, cut power to Board B. Accept when the car stops and latches a link fault within the timeout budget recorded here once it is chosen. | Video showing the power cut and the stop, plus the serial log. **Result:** Cutting power to Board B stops the car and latches FAULT_LINK_TIMEOUT (code 2); restoring power clears the bit on the first heartbeat. Demonstrated 08/21. | PASS |
| VV-14 system | Fall triggers stop: with the car moving, trigger a fall on Board B. Accept when the car stops and latches a fall fault, and the two faults are distinguishable in the log. | Video of the trigger and stop, plus the serial log showing the fault code. **Result:** A triggered fall stops the car and latches FAULT_FALL_DETECTED (code 4), distinct from the link timeout — the mechanism works. Across 20 drops the detection rate is 8 in 10, and this row accepts only a full rate. See the README for why the thresholds are unlikely to be the cause. | FAIL |
| VV-15 system | Control loop holds its deadline under load: run the background compute task at full rate and measure control period jitter. Accept when the jitter stays within the budget recorded here once it is chosen. | Captured jitter measurements with the load task confirmed running. **Result:** loadTask runs and the loop keeps working, but period jitter was never measured. | NOT TESTED |
| VV-16 system | Floor accuracy: command each floor ten times from different starting floors and measure the stopping error. Accept when every stop is within the tolerance recorded here once it is chosen. | Table of thirty commanded stops with measured error, plus a photo of the measurement method. **Result:** The car reaches and holds commanded heights and travels both directions repeatedly, but the thirty-stop error table was not collected. | NOT TESTED |

## Status values

- **NOT TESTED** -- no evidence collected yet. Every row starts here.
- **NOT MEASURED** -- the system was exercised and did not fail, but the
  quantity this row accepts on was never read. Distinct from NOT TESTED
  because the check is about a margin, and absence of failure is not a
  margin.
- **PARTIAL** -- evidence collected, criterion partly met. Nothing observed
  contradicts the full criterion; the full criterion was simply not run.
- **FAIL** -- evidence collected, acceptance criterion not met.
- **PASS** -- evidence collected, criterion met, and Hongyi has signed off.

## Outcome, 08/21/2026

7 PASS, 1 PASS (operation), 2 FAIL, 1 PARTIAL, 2 NOT MEASURED, 3 NOT TESTED.

The two failures are recorded as failures on purpose. VV-07 fails because
the row demands every driver report init success and we deliberately gave up
the driver's UART channel; VV-14 fails because the row demands a full
detection rate and we measured 8 in 10. Neither was rewritten to fit the
result. A row rewritten after the fact to match what happened is not a
check.
