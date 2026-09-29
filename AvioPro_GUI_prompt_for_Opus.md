# Prompt for Antigravity / Claude Opus — AvioPro Dual-Controller GUI Update

I have an existing dual-controller (Controller A + Controller B) ground station GUI for
a high-power rocketry avionics system called AvioPro, plus a screen for the CanSat
payload it carries. It talks to the rocket over LoRa, received by a ground station PCB
that CRC-verifies and cleans each packet before handing it to this GUI over serial —
you can trust whatever arrives on serial as already-validated. Right now only Start
TX / Stop TX actually work end to end; I need everything else wired up for an upcoming
competition where reliable, unambiguous GUI-driven command verification is the main
judged requirement.

**I'm attaching two reference images of the exact layout I designed (Ctrl_A/Ctrl_B
screen and Payload/CanSat screen).** Recreate this layout as closely as possible —
**prioritize matching this design over any existing layout in the current codebase.**
It's deliberately compact and built for fast visual interpretation mid-competition;
preserve that density and arrangement rather than reflowing it into something more
spacious. Panel positions, the tab bar, the state-arc diagram, the gauges, the graphs,
and the right-hand command column should all end up where they are in the images.
Where something below isn't shown in the images (noted per-item), add it in the
smallest, least disruptive way you can rather than restructuring the layout — ask me
first if you're not sure where it should go.

## Screen 1 — Ctrl_A / Ctrl_B / Payload tab bar, Avionics view

Layout, top to bottom / left to right, matching the image:

- Top bar: mission timer (`T: MM:SS:msms`), and a 3-way tab selector `Ctrl_A | Ctrl_B |
  Payload`. Ctrl_A and Ctrl_B share this same screen layout — see the note below on
  what changes between them.
- Left column: a vertical bar gauge labeled `H` / `H_MAX: 1000` (altitude), and two
  semicircular gauges: acceleration (`A_MAX`, with Ax/Ay/Az readouts) and velocity
  (`V_MAX`, with Vx/Vy/Vz readouts).
- Below those: a GPS block (`Lat / Lon / Alt / SatCnt`), a radio-link block
  (`Radio_connected / Rssi / SNR / Packet_loss`), an orientation block (`Gx/Gy/Gz` and
  `P/Y/R`), and a battery/environment block (`Batt / Temp / Pressure`).
- A scrollable **Debug** console. It shows raw incoming packets **and** any
  errors/warnings/flags the GUI itself raises (e.g. stale packet, command not acked,
  Controller B not present) — both streams interleaved into the same scrollback, not
  separate panels.
- Center top: the flight-state arc (`Launchpad Boot → Ascent → Payload_separation →
  Descent → Touchdown`), with `State: <current>` printed in the middle. Highlight
  wherever the current state sits on the arc.
- Center: two live line graphs — accelerometer XYZ (Ax/Ay/Az) and a second plot
  overlaying baro altitude vs. another altitude source (`H_baro`, `Alt`) — same colors/
  legend style as shown.
- Right column: the command button stack, top to bottom:
  `start_Tx, stop_Tx, Status, buzzer_play, calibrate, Servo_parachute, Servo_payload,
  Reset_controller`, then `Additional_options` which flies out a secondary panel with
  `Save_csv, Save_checkpoint, Show_map, Show_animation, save_data, Start_server,
  Stop_server`.

### Ctrl_A vs Ctrl_B

Same screen, same widgets. Controller B's telemetry doesn't populate every field A's
does — **for any field B doesn't send, keep showing Controller A's current value in
that slot** rather than a blank/zero, so the operator always sees a real number.

### Command behavior specifics

- **start_Tx / stop_Tx** — already working, keep as is. Visually distinguish "never
  started this session" from "stopped, can resume" if you can do it without adding a
  new element.
- **Status** — on-demand request for the live sensor-health/go-no-go packet (this is a
  structurally different downlink than telemetry, tagged with its own packet-type byte
  by the ground station — don't infer packet type from size or arrival order, use the
  tag). Can be requested any time, any number of times, in any flight state.
- **buzzer_play** — fires the locator buzzer briefly.
- **calibrate** — zeroes the altitude baseline to the current reading.
- **Servo_parachute / Servo_payload** — these are **one-shot, click-and-confirm
  buttons**, not separate open/close controls. Click → confirmation dialog → on
  confirm, send the single servo command. The avionics firmware does the full
  0°→90°→0° open-close cycle on its own — the GUI doesn't sequence that and doesn't
  need a second click to "close." Want it to move again? Click it again, confirm
  again. Make `Servo_parachute` visually distinct (e.g. warning color) since it's the
  parachute bay.
  > **Flag for you (Harshit):** this is a different command contract than what's
  > currently in the Controller A firmware I gave you earlier (which has separate
  > `PARACHUTE_OPEN`/`PARACHUTE_CLOSE`/`CANSAT_OPEN`/`CANSAT_CLOSE` commands). Building
  > the GUI this way means Controller A needs a follow-up change to two single
  > auto-cycle commands instead of four open/close commands — let me know if you want
  > me to make that change now or when we next touch the avionics code.
- **Reset_controller** — confirmation dialog required. Firmware only honors this from
  `LAUNCH_PAD` state — if the GUI knows current state, gray the button out or warn the
  operator outside that state rather than letting them send a command that'll be
  silently ignored.
- **Additional_options flyout:**
  - `Save_csv` — exports raw logged data as CSV, nothing else.
  - `Save_checkpoint` — saves current session state.
  - `Show_map` — live view only, not a save action.
  - `Show_animation` — live view only, not a save action.
  - `save_data` — this now bundles what used to be separate "download animation,"
    "graphs," and "PDF report" actions into one export: clicking it generates and
    saves the animation export, the graph exports, and a PDF report together in one
    action.
  - `Start_server` / `Stop_server` — unchanged.
- **Missing from this screen entirely: setting the sea-level pressure (QNH) the
  avionics uses for altitude, with the built-in fallback shown if unset.** This isn't
  in the image — add a small numeric input for it wherever it fits with the least
  disruption (near `calibrate` is a reasonable default, but use your judgment or ask).

## Screen 2 — Payload (CanSat) view

This is visually different from the avionics screen on purpose — CanSat's whole job is
descending to one target point, not tracking a full flight, so the layout drops the
big altitude/velocity gauges in favor of a tighter, descent-focused set of widgets:

- Same top tab bar (`Ctrl_A | Ctrl_B | Payload`, `Payload` selected) and timer.
- Same flight-state arc as the avionics screen (shared mission timeline).
- Radio-link block (`Radio_connected / Rssi / SNR / Packet_loss`) and a scrollable
  Debug console — same behavior as screen 1 (raw packets + GUI-raised errors/flags
  interleaved).
- Two orientation/accel line graphs (Pitch/Yaw/Roll, and Ax/Ay/Az) instead of the
  avionics altitude/accel pair.
- A GPS/position block (`H, Lat, Lon, Alt, SatCnt, Heading`) and, taking up the large
  central-right space, a live 3D plot or an annotated live path over a GPS map —
  whichever is more practical to implement, per the "3D plot / or direct annotate path
  on gps map live" note in the mock.
- Right column command stack: `start_Tx, stop_Tx, Status, buzzer_play, calibrate,
  Enable Servos, Servo_L_+ve, Servo_L_-ve, Servo_R_+ve, Servo_R_-ve,
  Reset_controller, Additional_options` (same flyout contents as screen 1: `Save_csv,
  Save_checkpoint, Show_map, Show_animation, save_data, Start_server, Stop_server`).

### Payload servo control — different confirmation model than avionics

- **Enable Servos** is a **toggle**, and it's the confirmation gate for the whole
  group: click it → confirm once → all four `Servo_L_+ve / Servo_L_-ve / Servo_R_+ve /
  Servo_R_-ve` buttons become active and can be clicked freely without a confirmation
  dialog per click while enabled. Click **Enable Servos** again to disable — no
  confirmation needed to turn it off, and the four buttons go back to inactive/locked
  until re-enabled (and re-confirmed).
- Each directional button sends a fixed-increment nudge with no value/angle in the
  payload — the increment amount lives in the CanSat firmware, not the GUI.

### Missing from this screen — not in the mock, still required

These were part of the payload command set I specified earlier but aren't drawn in the
mock. Add them without disrupting the layout above — a small additional row, a
collapsible section, or folded into `Additional_options` are all reasonable; use your
judgment or ask:
- **Set target GPS** — lat/lon numeric input.
- **Set PD + fuzzy controller constants** — a small form; exact parameter names/count
  are still TBD on my end, stub it with placeholder fields for now.
- **Spiral Down** — an immediate-touchdown override for when the CanSat is drifting
  too far. This is rare-use and high-consequence: give it its own clearly separate,
  confirmation-gated control, visually set apart from the routine steering buttons —
  don't fold it into the directional-nudge group.

## General requirements

- This is judged live in front of officials — the operator should never be unsure
  whether a command was sent, acknowledged, or ignored. If the protocol gives you
  enough to infer that (sequence numbers exist uplink-side), show pending/sent/acked
  state per command.
- Every destructive or irreversible action needs a confirmation dialog: both servo
  commands (per their respective models above), Reset_controller, and Spiral Down.
- Keep the existing working start_Tx/stop_Tx code path intact; extend rather than
  rewrite unless matching the layout above genuinely requires restructuring.

Ask me anything that's unclear before you start restructuring the codebase.
