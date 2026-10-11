# TODO / Known Issues

Findings from the review done when the project was ported from the Arduino IDE
to PlatformIO. Fixed items are kept as a record. Open items need validation on
hardware or a decision, so they were not changed.

## Fixed in 1.9 (build verified only, not tested on hardware)

- **`GMAX` / `GMIN` called `setLimit`** instead of `getLimit` ([src/MFT.cpp](src/MFT.cpp)).
- **`SGRD` / `SGRDA` ignored the `cond|a|b` form**: a missing `else` meant the
  value was always re-parsed as a plain number afterward.
- **`TRIG1` + `TWALT2` + `CHANGE` toggled channel 1** (copy/paste, `toggleTWaltV(0)`).
  The two trigger ISRs were merged into one `trigISR()`.
- **`SCLOCK,0` divided by zero** (`1000000/(2*freq)`).
- **`TRIGOUT,<bad arg>` sent NAK then ACK.**
- **`RESET` only restarted if pin 2 (Trig2) was high**, now restarts always.
- **Startup passed the `Fwd` array address as the direction flag**, so reverse
  was never restored after power up.
- **Frequency could reach 0 at startup** (divide by zero in the timer period).
  Period calculation is now shared and clamps to 1 Hz.
- Ported-from-Arduino build errors: missing includes, missing prototypes, missing `lib_deps`.
- Removed declarations that had no definition (`ReadADC`, `MonitorFlag`,
  `ProgramFLASH`, `ProgramGOTO`, `LoadAltRev`, `WhereAmI`, `EraseUpper`).

## Open – needs hardware validation

1. **Race between ISR and command handlers.** `Timer1ISR` reads `twave[]`,
   `Open[]` and `openMask[]` while `defineTWvector()` rewrites `twave[]` from
   the main loop, so a glitched state can reach the switches. The trigger ISRs
   also write `Fwd`/`Open` without calling `defineTWvector()` (the
   direction trigger changes the flag but the table is not rebuilt – confirm
   whether that is intended). `AtomicBlock.h` exists but is unused and has no
   Teensy 4 port; use `noInterrupts()` / `__disable_irq()` or double-buffer the table.
2. **SPI and I²C from interrupt context.** `Timer1ISR` does an SPI transfer;
   `Trig*isr` call `MAX5815` over I²C (`setTWaltV`) and `delayMicroseconds`.
   I²C from an ISR can collide with a main-loop transfer. Consider setting a flag
   and doing the DAC write in `loop()`.
3. **`advanceCounter()` can be called from an ISR and the clock timer** and does
   `digitalWrite` + `delayMicroseconds(5)`; `pulseCounter.count` is not atomic.
4. **`SPI.beginTransaction()` is never ended** in `MAX14802()`; fine while the
   MAX14802 is the only SPI device. Revisit if another device is added.
5. **`SetGRDvoltage` linearization** uses fixed constants (3.888, 0.7553,
   0.9516) that bypass the stored calibration; `SGRDA` supersedes it.
6. **Frequency limits.** `MINfrequency` is 0; `SFREQ` clamps to 1 Hz but `SMIN,FREQ,0`
   is still accepted. With 8 states per cycle and a µs timer the actual
   frequency is quantized (see `GAFREQ`); at 25 kHz the period is 5 µs.
7. **Phase shift** is stored as any int and applied as `(deg / 45) & 7`, so
   values that are not multiples of 45 truncate toward zero (e.g. 30 gives no
   shift). Consider validating or rounding when stored.
8. **`SetTWAvoltage`** rejects out-of-range values while `SetTWvoltage` clamps.
9. **`FormatFLASH()`** (`FORMAT`) is a stub that only ACKs.
10. **`ERR_EEPROMWRITE` and `ERR_ETHERNETCOMM` are both 110** in
    [include/Errors.h](include/Errors.h) (shared header); and `RESTORE` reports a
    write error when it is really a read/signature failure (`ERR_EEPROMREAD`).
11. **`Token` buffer is 20 characters** (`MaxToken` in Serial.cpp). Longer tokens
    are silently truncated, including conditional expressions such as
    `STWV,1,>25|ETRGCMD2|+=1` (they are sent through `STRGCMDn` long-string mode
    so are safe there, but not as direct arguments).
12. **`SNAME`/`CMDstr` set** does `strcpy` with no length check (name is 20 bytes).
13. **`UserInput()` blocks forever** (used by `CAL`) and the calibration code
    does not validate `V2 != V1`.
14. **Limits** `setLimit` validates against compile time `MINvoltage`/`MAXvoltage`
    only; it does not check min <= max.

## Cleanup / housekeeping

- `src/build/` is stale Arduino IDE output and can be deleted. `firmware/MFT.ino.TEENSY40.hex`
  is byte-identical to `MFT_V1.2.hex` and can be deleted.
- [src/Hooks.c](src/Hooks.c), `mySysTickHook` and `msTimerIntercept` are left over
  from the SAMD (M0) code and do nothing on Teensy; candidates for removal.
- `ReadAllSerial()`, `Update()` and other handlers could move out of `MFT.cpp`
  (1,400 lines) into `Triggers.cpp`, `Calibration.cpp` and `Commands.cpp` as in DMS.
- `Hardware.cpp` CRC helpers and `AtomicBlock.h` are unused.
- The `GetToken(String, int)` overload in Serial.cpp is unused.
- No unit tests (`test/` is empty). The pure logic (`checkIF`, `checkChange`,
  `checkPattern`, `defineTWvector`, ring buffer) could be tested on the `native` platform.
