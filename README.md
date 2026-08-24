# ArduinoServoController-UART-InMoov

Drive the servos of an [InMoov](http://inmoov.fr/) robot with short text commands over a
serial port.

Send `HH50` to center the head, or `RI+10` to curl the right index finger a little
further. The Arduino handles the pulse-width math, enforces each servo's safe range, and
powers down servos that have gone idle — so the host driving it (a Raspberry Pi, a laptop,
anything that can open a serial port) never needs to know servo geometry.

Positions are percentages of each joint's configured safe range rather than raw degrees.
That means a host can say "close the hand" without knowing that the right middle finger is
mounted backwards and travels 40°–140°, and you can recalibrate any joint without
touching a line of host code.

Written for "Gary," an InMoov robot — but there's nothing InMoov-specific in the design.
Any project driving hobby servos through a PCA9685 can use it by editing one table.

> [!WARNING]
> Servos are strong enough to destroy 3D-printed parts and strip their own gears. Bring up
> new joints one at a time with a deliberately narrow range, and widen it gradually. See
> [Calibrating a servo](#calibrating-a-servo).

## Contents

- [Hardware](#hardware)
- [Installing](#installing)
- [First test](#first-test)
- [The serial protocol](#the-serial-protocol)
- [Default channel map](#default-channel-map)
- [Configuring servos](#configuring-servos)
- [Driving it from a host](#driving-it-from-a-host)
- [Idle power-down](#idle-power-down)
- [Changes in this revision](#changes-in-this-revision)
- [Known limitations](#known-limitations)

## Hardware

- An Arduino — developed against and compile-tested on the **Uno**. It needs only I²C and
  one hardware serial port, so most boards will do. On an Uno the sketch uses about 8.4 KB
  of flash (26%) and 1.4 KB of RAM (69%), leaving roughly 620 bytes for the stack. Fine as
  shipped, but worth watching if you add a lot more servos — each row of the configuration
  table costs another 14 bytes of RAM.
- One or two [Adafruit 16-Channel 12-bit PWM/Servo Driver](http://www.adafruit.com/products/815)
  boards, at I²C addresses `0x40` (channels 0–15) and `0x41` (channels 16–31). One board is
  fine — leave the second board's channels out of the table.
- Hobby servos, and a servo power supply that can actually feed them. Servos must **not**
  be powered from the Arduino's onboard regulator; a hand full of servos moving at once
  will brown it out.

### Dependencies

- [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
- `Wire` (bundled with the Arduino IDE)

## Installing

The Arduino IDE requires a sketch's folder name to match its `.ino` filename, and this
repository's name doesn't. Clone it into a folder named `UART_InMoov`:

```bash
git clone https://github.com/Lukeluca/ArduinoServoController-UART-InMoov.git UART_InMoov
```

Then open `UART_InMoov/UART_InMoov.ino`, install the Adafruit library via
**Tools → Manage Libraries**, and flash.

## First test

Before wiring a single servo, flash the sketch and open the Serial Monitor at
**115200 baud** with the line ending set to **Newline**. You should see:

```
M:InMoov Arduino Online
```

Type `HH50` and press enter. Even with nothing connected you'll get an acknowledgement:

```
C:HH50
M:Received command - HH value 50%
```

That confirms the protocol works before mechanical problems can confuse the picture. Now
connect one servo, put it on a channel in the table, and try `HH0`, `HH50`, `HH100`.

## The serial protocol

Open the port at **115200 baud**. Commands are newline-terminated (`\n`). Several may be
sent on one line, separated by spaces, and are applied in order.

```
<CODE><VALUE>
```

- **`CODE`** — a two-letter servo abbreviation from the table below.
- **`VALUE`** — either an **absolute** position from `0` to `100`, as a percentage of that
  servo's configured safe range, or a **relative** adjustment from `-100` to `+100` when
  prefixed with an explicit `+` or `-`.

### Examples

```
HH50                     move the head to the center of its horizontal range
HV20                     tilt the head to 20% of its vertical range
RI+10                    open the right index finger a little further
RI-25                    curl the right index finger 25% further closed
RT0 RI0 RM0 RR0 RP0      close the whole right hand at once
DP                       send every enabled servo to its default (50%) position
```

A relative command for a servo that hasn't been positioned since power-on moves it to 50%
first, then applies the adjustment from there.

### Servo codes

| Code | Servo | Code | Servo |
|------|-------|------|-------|
| `HH` | Head Horizontal (rotation) | `LT` | Left Thumb |
| `HV` | Head Vertical (neck) | `LI` | Left Index |
| `HM` | Mouth | `LM` | Left Middle |
| `RB` | Right Bicep Twist | `LR` | Left Ring |
| `RW` | Right Wrist Twist | `LP` | Left Pinky |
| `RT` | Right Thumb | `LW` | Left Wrist Twist |
| `RI` | Right Index | | |
| `RM` | Right Middle | `DP` | *(all)* Default Positions |
| `RR` | Right Ring | | |
| `RP` | Right Pinky | | |

The six left-arm servos ship **disabled** — see [Configuring servos](#configuring-servos).

> [!NOTE]
> For the fingers, **`0` is always closed and `100` is always open** — every
> finger, whichever way its servo happens to be mounted. That is what the
> `inverted` column is for: it absorbs the mounting inside the firmware, so a
> host never has to know that the right middle finger is built backwards. Three
> of the examples above had this reversed in an earlier revision of this file.

### Responses

Every line the Arduino sends is prefixed, so a host can parse it line by line:

| Prefix | Meaning |
|--------|---------|
| `M:` | Informational message |
| `C:` | Echo of the raw command line as received |
| `E1xx:` | Error |

| Error | Meaning |
|-------|---------|
| `E100` | Unsupported servo — no such two-letter code |
| `E101` | Missing value — no digits followed the servo code |
| `E102` | Servo not enabled — present in the table, but not marked `enabled` |
| `E103` | Malformed command — shorter than a two-character servo code |

Command lines are limited to 31 characters. Longer input is truncated rather than
rejected, so keep host-side batches short.

## Default channel map

These are the PCA9685 channels as shipped, reflecting one particular robot's wiring.
**Change the `pin` values in the table to match how you actually wired yours.**

| Board | Channel | Code | Servo |
|-------|---------|------|-------|
| `0x40` | 0 | `HH` | Head Horizontal (rotation) |
| `0x40` | 1 | `HM` | Mouth |
| `0x40` | 2 | `RB` | Right Bicep Twist |
| `0x40` | 3 | `RT` | Right Thumb |
| `0x40` | 4 | `RI` | Right Index |
| `0x40` | 5 | `RM` | Right Middle |
| `0x40` | 6 | `RR` | Right Ring |
| `0x40` | 7 | `RP` | Right Pinky |
| `0x40` | 8 | `HV` | Head Vertical (neck) |
| `0x40` | 9 | `RW` | Right Wrist Twist |
| `0x40` | 15 | `LT` | Left Thumb |
| `0x41` | 16 | `LI` | Left Index |
| `0x41` | 17 | `LM` | Left Middle |
| `0x41` | 18 | `LR` | Left Ring |
| `0x41` | 19 | `LP` | Left Pinky |
| `0x41` | 20 | `LW` | Left Wrist Twist |

Note that this default map splits the left hand across both boards, with the left thumb
alone on the first board.

## Configuring servos

Every servo is one row of the `SERVOS[]` table near the top of `UART_InMoov.ino`. Adding a
servo, recalibrating one, or enabling the left arm means editing that table and nothing
else — there are no parallel lookup functions to keep in sync.

```c
//  code  pin              minDeg maxDeg  minPulse  maxPulse  inverted  enabled
{  "HH",  PIN_HEAD_TWIST,  40,    100,    180,      580,      false,    true  },
```

| Field | Meaning |
|-------|---------|
| `code` | Two-letter UART command code |
| `pin` | PCA9685 channel (`0`–`15` on board `0x40`, `16`–`31` on board `0x41`) |
| `minDeg` / `maxDeg` | Safe travel limits in degrees. Command value `0` maps to `minDeg`, `100` to `maxDeg`. **This is the guard rail that stops a servo tearing a printed part apart** |
| `minPulse` / `maxPulse` | This servo's calibrated pulse lengths (out of 4096) for 0° and 180°. `150`/`600` is a reasonable starting guess |
| `inverted` | `true` if the servo is mounted backwards, so command `0` should drive the opposite pulse. Which physical end that is depends on the mounting — the flag exists to make a backwards-mounted joint agree with the others, not to name an end |
| `enabled` | `false` for servos not yet wired or calibrated. They are never driven, and commands targeting them return `E102` |

### Calibrating a servo

1. Add a row with `enabled: false`, `minPulse`/`maxPulse` of `150`/`600`, and a
   deliberately narrow `minDeg`/`maxDeg` — say `80` to `100`.
2. Flip `enabled` to `true`, flash, and step through `0`, `50`, `100` for that code.
3. Widen the degree range a few degrees at a time, re-testing, until the joint reaches its
   true mechanical limits without straining or buzzing at the endpoints.
4. If the joint moves opposite to what you expect, set `inverted: true` rather than
   swapping `minDeg` and `maxDeg` — inversion is handled correctly for relative moves too.

One servo at a time. A mis-set range is the fastest way to strip a gear.

## Driving it from a host

Any language that can open a serial port works. In Python, with
[pyserial](https://pyserial.readthedocs.io/):

```python
import serial
import time

gary = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
time.sleep(2)  # opening the port resets most Arduinos; wait for the boot message

def send(command):
    gary.write((command + "\n").encode())
    time.sleep(0.05)
    while gary.in_waiting:
        print(gary.readline().decode(errors="replace").strip())

send("DP")             # everything to its default position
send("HH50 HV50")      # head centered
send("RI+15")          # curl the right index finger
```

Two things to watch for: opening the port resets the board on most Arduinos, so wait for
`M:InMoov Arduino Online` (or just sleep) before sending; and check returned lines for an
`E1xx` prefix, since an unrecognized command fails quietly as far as the mechanism is
concerned.

## Idle power-down

A servo that receives no command for 5 seconds (`ADAHAT_IDLE_TIMEOUT_MS` in `AdaHat.h`)
has its PWM output switched off. This stops the buzzing and current draw of a servo
straining to hold a position nobody is asking for any more.

There's a trade-off to know about: a powered-down joint can be back-driven by gravity,
while the firmware still believes it's where it was last commanded. A later **relative**
move is therefore calculated from a position the joint may no longer be in. If that
matters for your build, send an absolute position after an idle period, or raise the
timeout.

## Changes in this revision

If you're running an earlier version, this one fixes several real bugs:

- **`DP` crashed the sketch.** The default-positions command was parsed for a numeric
  value it never has, dereferencing a null pointer.
- **`DP` also read past the end of its array**, driving PWM channels from whatever
  happened to be adjacent in memory. (`sizeof()` on an `int` array is a byte count, not an
  element count.)
- **Relative moves on inverted servos jumped nearly full range.** A `+10` nudge was
  transformed into `100 - 10`, so an inverted joint slammed to 90% instead of easing 10%.
- **Servos on the second PWM board never powered down.** The idle loop stopped at channel
  16, making its own second-board branch unreachable.
- **The idle loop flooded the I²C bus**, re-sending a shutoff to every idle channel on
  every pass of the main loop. It now sends one shutoff per idle period, and skips
  channels with no servo configured.
- Channel indices are bounds-checked, out-of-range command values are clamped before they
  can wrap, idle timing survives the `millis()` rollover, and malformed short commands
  return `E103` instead of reading stale buffer bytes.

Configuration also moved into the single `SERVOS[]` table described above, replacing three
parallel lookup functions that had to be kept in step by hand. Inversion is now per-servo,
so more than one servo can be mounted backwards.

## Known limitations

- **No motion smoothing.** Servos move to their target as fast as they can, which looks
  abrupt and causes current spikes when several move together. Worth knowing if you plan
  to drive this from something like a vision tracking loop.
- **`DP` moves every enabled servo simultaneously**, a meaningful inrush on a marginal
  supply.
- **Positions are commanded, not measured.** Standard hobby servos report nothing back, so
  the firmware tracks what it asked for, not where the joint actually is.
- **The head's vertical range is capped below its mechanical maximum** to avoid straining a
  camera cable. Raise `HV`'s `maxDeg` to suit your own build.
- **An unknown code with no digits reports `E101`** (missing value) rather than `E100`
  (unsupported servo), because the value is parsed first.

## Contributing

Issues and pull requests welcome. If you're adding servos or another board, please keep
configuration in the `SERVOS[]` table rather than adding parallel lookup functions.

Thanks to [@2hands10fingers](https://github.com/2hands10fingers) for multi-board support.

## License

Released under the MIT License — see [LICENSE](LICENSE).
