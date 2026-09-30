Run from the repository root:

```powershell
& motionsimbot/tests/pid-regression.ps1
node motionsimbot/tests/webui.test.cjs
```

The PID harness extracts the production control and tuning blocks and compiles
them through PowerShell Add-Type with C# math and platform adapters. It tests
angle wrapping, repeated modes, calibration isolation, PWM-driven motor models,
output limits, tuner duration, acceptance and rollback. It is not a native C,
hardware, sensor timing or concurrency test. ESP-IDF builds check native C
compilation separately. The Web UI test executes the embedded JavaScript with
stubbed DOM and telemetry responses.

On hardware, verify the v0.4 badge, then test 110 to 10, 160 to 110 and 350 to 10.
Record target, profiled target, actual angle, PWM, polarity and E-stop reason if
movement fails. A successful tune takes at least 49 seconds, tests offsets of
5, 10 and 15 degrees in both directions, and validates candidate gains. Failed
or interrupted tuning restores the previous gains. Successful gains are active
in RAM until Save to Flash is selected. Tuning requires an already stable
controller; it refines gains heuristically and is not motor identification.

v0.4 adds Encoder and Control Diagnostics. The stationary check samples raw
angle and real AS5600 status approximately every 100 ms, with motors disarmed.
Its 0.5-degree warning is a diagnostic threshold, not a sensor specification.
It cannot rule out fast noise between samples or angular nonlinearity.
Download Diagnostics includes that check and the last 512 Joint 1 PID samples
(about 5 seconds at the nominal loop rate). The trace freezes at calibration
failure or E-stop until another target or calibration starts. It records logical
and physical PWM commands, not measured bridge voltage or current. Export it
before another command. Sensor sample age measures successful angle reads;
cached angles no longer count as proof of fresh feedback.
