# Cobra development guide

## Purpose

Cobra is a lightweight, approachable successor to Gepard. It will operate a
drone and grow one small, understandable feature at a time. Its purpose is
both useful flight software and a learning project: each implemented service
must make its responsibilities and failure modes clear enough for its operator
to diagnose problems independently.

Gepard is a reference implementation, not an architectural template. Reuse a
Gepard idea only after reducing it to the smallest form that satisfies Cobra's
current milestone.

## Current milestone

Implement only these capabilities, in order:

1. Communicate with a flight controller over MAVLink.
2. Record MAVLink telemetry with a basic logging system.

Do not add mission execution, mapping, object detection, payload release,
camera handling, or future-service abstractions until a later milestone
explicitly requires them.

## Implementation rules

- Add one feature at a time. Each feature must build, run, and be verifiable
  before beginning the next one.
- Keep the directory structure and dependency graph small. Add a file, layer,
  interface, configuration option, or dependency only when the current
  capability needs it.
- Prefer direct, readable control flow and names that explain intent. Avoid
  speculative abstractions and compatibility layers.
- Keep source comments rare. Write a comment only for a non-obvious constraint,
  safety reason, protocol detail, or decision that cannot be expressed through
  code structure and naming.
- Put explanations of behaviour, data flow, MAVLink details, configuration,
  operating steps, and troubleshooting in `docs/`. Documentation must explain
  the code that exists and be updated in the same change.
- Keep documentation focused on why and how the current implementation works;
  do not duplicate code line by line.
- Treat live-flight interaction carefully. Begin with passive observation and
  logging unless a later task explicitly authorizes commands to the vehicle.
- Use Gepard to understand MAVLink behaviour and operational lessons, while
  avoiding copying its file layout or comment-heavy style.
- Provide a small reproducible verification path for each feature, preferably
  using a simulator before hardware.

## Decisions for the first implementation

- Use C++17 and CMake.
- Develop on a laptop first and keep the build portable to Jetson Orin.
- Start with ArduPilot SITL and Mission Planner. Cobra connects to the MAVLink
  endpoint configured in TOML; the default mirrors Gepard's simulator router:
  `tcp:127.0.0.1:5762`.
- Use the MAVLink 2 wire protocol and the `ardupilotmega` dialect. Gepard
  includes the generated `ardupilotmega` headers and uses the same TCP endpoint
  for its simulator profile. Its later support for UDP and serial is out of
  scope for this milestone.
- Cobra sends its own standard MAVLink heartbeat. After an autopilot heartbeat,
  it requests the agreed telemetry messages with `MAV_CMD_SET_MESSAGE_INTERVAL`.
  It must not send vehicle-control commands.
- Initially request and record `HEARTBEAT`, `SYS_STATUS`, `GPS_RAW_INT`,
  `GLOBAL_POSITION_INT`, `ATTITUDE`, `VFR_HUD`, and `BATTERY_STATUS`.
- Logs are operator-readable. Each program run creates
  `cobra_flights/YYYY_MM_DD_HH_MM_SS/` below the repository, with the telemetry
  log and a concise operational log inside it.
- Configuration is TOML. Keep separate example configuration and setup guides
  for laptop/SITL and Jetson, even while only the laptop path is implemented.
- Every milestone includes a tutorial-style document covering the protocol
  concepts, data flow, operating steps, and troubleshooting.

## Required decisions before implementation

Decide the exact endpoint published by the local ArduPilot SITL setup before
running Cobra. Mission Planner is a ground-control client, so it may share a
MAVLink endpoint only when the SITL launcher or router explicitly exposes one
for Cobra.
