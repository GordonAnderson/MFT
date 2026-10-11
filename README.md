# MFT

Firmware for the MFT (multi function Twave generator) board. It generates two
independent travelling-wave (Twave) switch sequences for an ion guide, with
programmable amplitude, frequency, direction, phase and bit pattern, plus a
programmable guard voltage. Two trigger inputs, a trigger output, a pulse
counter and a clock output allow the waveform to be modified in real time or to
run stored command strings.

Current version: **1.9** (see the revision history at the top of
[src/MFT.cpp](src/MFT.cpp)).

## Hardware

- **MCU:** NXP i.MX RT1062 on a Teensy 4.0 (`teensy40` environment).
- **Twave switches:** Microchip MAX14802 high voltage switches on SPI
  (mode 0, 20 MHz), latch on pin 10, clear on pin 14. Each channel is 8 bits,
  the upper byte is the complement of the lower so each pair is driven
  push-pull.
- **DACs:** Maxim MAX5815 on I²C (address `MAX5815add`), channel 0 = TW1
  amplitude, 1 = TW2 amplitude, 2 = guard. LDAC = pin 5, CLR = pin 4.
- **Readback:** processor ADC, TW1 = A7, TW2 = A6, guard = A8, 12 bit,
  averaged and low pass filtered in a 25 mS thread.
- **Digital I/O:** TrigOut = pin 9, Trig1 = pin 3, Trig2 = pin 2.
- **Serial:** USB (`Serial`) and a hardware UART (`Serial1`, baud set with
  `SBAUD`, read at startup). Replies go to whichever port last sent data.

## Source layout

| File | Purpose |
|------|---------|
| [src/MFT.cpp](src/MFT.cpp) | `setup()`/`loop()`, twave generation, command handlers, triggers, counter, clock, calibration |
| [src/Serial.cpp](src/Serial.cpp) | Ring buffer, tokenizer, command table (`CmdArray`) and dispatcher |
| [src/Hardware.cpp](src/Hardware.cpp) | MAX14802 / MAX5815 drivers, ADC and DAC count conversion |
| [include/MFT.h](include/MFT.h) | Pins, `MFTdata` settings structure, shared declarations |
| [include/Serial.h](include/Serial.h) | Command table types, ring buffer, ACK/NAK macros |
| [include/Errors.h](include/Errors.h) | Error codes (shared with other MIPS modules) |
| [include/AtomicBlock.h](include/AtomicBlock.h) | Interrupt-lock helpers (not currently used) |

## How it works

A `Timer1` interrupt runs at 8 × the Twave frequency. Each tick outputs the next
of 8 states from the `twave[ch][8]` table to the MAX14802. The table is built by
`defineTWvector()` from the 8 bit pattern, direction (forward rotates right,
reverse left) and the phase shift (in 45° steps). An open mask can force
selected switch pairs off.

The main loop moves serial characters into a 4 kB ring buffer and executes
complete lines. A `ThreadController` thread (`Update`, 25 mS) filters the
readbacks. Settings (`MFTdata`) are saved to and restored from EEPROM with
`SAVE` / `RESTORE`; they are valid when the signature `0xAA55A5A5` matches,
otherwise the defaults in `Rev_1_mftdata` are used.

## Serial protocol

Commands are comma separated, terminated by newline or `;`. `S` commands set,
`G` commands get. A successful set replies with ACK (0x06); errors reply with
NAK `0x15 ?` and the code is readable with `GERR`. `GCMDS` lists every command.
`MUTE,ON|OFF` suppresses replies, `ECHO,TRUE|FALSE` echoes commands.

Channel arguments are 1 or 2.

| Command | Description |
|---------|-------------|
| `GVER`, `GNAME`, `SNAME`, `GSTATUS`, `GERR` | Version, name, status (Running/Stopped/Stepping), last error |
| `SAVE`, `RESTORE`, `RESET`, `SBAUD`, `GBAUD` | Settings and system |
| `SFREQ`, `GFREQ`, `GAFREQ` | Twave frequency per channel, requested and actual |
| `SFWD`, `GFWD` | Direction `TRUE` forward / `FALSE` reverse, per channel |
| `SPTRN`, `GPTRN` | 8 bit binary pattern, per channel |
| `SFWDPS`, `GFWDPS`, `SREVPS`, `GREVPS` | Forward / reverse phase shift in degrees |
| `START`, `STOP`, `STEP,n` | Start/stop the waveform, or move n cycles |
| `STWV`, `GTWV`, `GTWVA` | Twave voltage set / get / readback, per channel |
| `STWVALT`, `GTWVALT` | Alternate twave voltage (selected by a trigger) |
| `SGRD`, `SGRDA`, `GGRD`, `GGRDA` | Guard voltage; `SGRDA` corrects below 5 V using readback |
| `SOPEN`, `GOPEN`, `SOMSK`, `GOMSK` | Output open enable and mask, per channel |
| `SMAX`, `GMAX`, `SMIN`, `GMIN` | Limits: `TWV,chan,value`, `GRD,value`, `FREQ,value` |
| `TRIG1`, `TRIG2` | `mode,function` – see below |
| `GTRIGIN,n` | Read trigger input 1 or 2 |
| `TRIGOUT` | `HIGH`, `LOW` or `PULSE` |
| `GCNT`, `CLRCNT`, `SCNTTRG`, `GCNTTRG` | Pulse counter and terminal count |
| `STRIGCNT`, `STRIGRST`, `STRIGCMD` (and `G…`) | Terminal count actions: pulse TrigOut, reset, run command string |
| `SCLOCK`, `GCLOCK`, `SCLKFUN`, `GCLKFUN` | Clock output 0–10000 Hz; function `NA`, `TRG` (TrigOut) or `CNT` (counter) |
| `STRGCMD1/2`, `GTRGCMD1/2`, `ETRGCMD`, `ETRGCMD1/2`, `SCMD`, `GCMD` | Two stored command strings and which is active |
| `CAL` | Interactive two point calibration of TW1, TW2 and guard |
| `DELAY`, `THREADS`, `STHRDENA`, `ECHO`, `MUTE` | Utility |

### Triggers

`TRIG1,mode,function` / `TRIG2,mode,function`

- mode: `POS`, `NEG`, `CHANGE`, `NA` (disable)
- function: `REV1`, `REV2` (direction follows the level), `OPEN1`, `OPEN2`,
  `CMD` (run the active command string), `CNT` (advance the counter),
  `TWALT1`, `TWALT2` (use the alternate voltage; `CHANGE` toggles it)

### Value expressions

`STWV`, `SGRD`, `SGRDA` and `SFREQ` accept a plain value, a relative change
(`+=2`, `-=0.5`) or a conditional `cond|true|false` where cond is `>n`, `<n`,
`>=n`, `<=n` or `=n` and each action is a value, a `+=`/`-=` change, or a
command (commas written as `_`). Example: `STWV,1,>25|ETRGCMD2|+=1`. These are
used by the stored command strings to build self-running sequences.

## Build

PlatformIO, environment `teensy40`:

```
pio run              # build
pio run -t upload    # build and flash (Teensy loader)
```

Libraries (fetched from `lib_deps`): GAACE_Core (`SerialBuffer`, `ThreadController`
glue), ArduinoThread, Adafruit_DotStar, TimerOne. Every build copies the
result to `firmware/MFT_V<version>.hex` (version parsed from `Version[]` in
`src/MFT.cpp` by [publish_firmware.py](publish_firmware.py)); earlier releases
(V1.2 – V1.8) are kept there too.

See [TODO.md](TODO.md) for known issues and open items.
