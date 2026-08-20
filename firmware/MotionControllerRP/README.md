# MotionControllerRP -- pico-sdk + FreeRTOS port

This is a port of the original PlatformIO / arduino-pico project to plain CMake +
pico-sdk + FreeRTOS, targeting the Raspberry Pi Pico 2 (RP2350). See
[`lib/README.md`](lib/README.md) for the three git submodules you need before building,
and build instructions.

## What changed, and why

**The two bare-metal busy loops are now FreeRTOS tasks.** `main_core0()`
(command-parser + path-planner) and `main_core1()` (servo control) used to be
un-yielding `while(true)` loops, one launched via arduino-pico's `setup()`/`loop()` on
core 0, the other via `multicore_launch_core1()` on core 1. They're now two FreeRTOS
tasks (`command_task` / `servo_task` in `src/main.cpp`), each pinned to its own core
with `vTaskCoreAffinitySet()`, running under the dual-core SMP FreeRTOS port
(`RP2350_ARM_NTZ`, from the `raspberrypi/FreeRTOS-Kernel` fork). This gives you actual
task/scheduler visibility (stack high-water marks, the ability to add more tasks
later, mutexes/queues that interoperate with `pico_sync`, etc.) instead of the cores
being permanently and opaquely owned by one loop each.

**Both tasks still run essentially flat-out**, matching the original behaviour: each
loop iteration ends with `taskYIELD()` rather than a real `vTaskDelay()`, so no
artificial rate limit is imposed on either the command-parsing/path-planning loop or
the servo control loop's update rate (the servo loop still computes its own `dt` from
`time_us_64()` every iteration, exactly as before). `taskYIELD()` just gives the
scheduler on that core a chance to run the idle task / timer service / anything else
you add later, without slowing this one down.

If you'd rather have the servo loop run at a fixed, deterministic rate instead (e.g.
to guarantee CPU headroom on core 1 for future tasks, at the cost of some peak
throughput), there's a comment in `servo_task()` in `main.cpp` showing how to swap the
`taskYIELD()` for a `vTaskDelayUntil()` against a fixed period.

**Arduino-framework dependencies were replaced with a small compatibility layer**
(`src/compat/`), rather than rewriting the control/kinematics/servo code itself:

| Arduino API | Replaced by | Backed by |
|---|---|---|
| `Serial` | `compat/Serial_compat.*` | pico-sdk stdio over USB CDC |
| `Wire` / `TwoWire` | `compat/Wire_compat.*` | pico-sdk `hardware_i2c` |
| `LittleFS` / `File` | `compat/LittleFS_compat.*` | [pico-vfs](https://github.com/oyama/pico-vfs) (POSIX `fopen`/`opendir`/`stat` on littlefs-over-flash) |
| `delay()` / `delayMicroseconds()` | `compat/Arduino_compat.h` | `sleep_ms()` / `sleep_us()` |

This was a deliberate choice: everything else in this codebase (SPI encoders, the
PWM/PIO motor driver, hardware timers, the multicore spinlocks) was already written
directly against pico-sdk, not Arduino, so the actual Arduino surface area in use was
small and self-contained. Shimming it kept the diff to the control/robotics logic
essentially zero, versus a much riskier full rewrite of files that are hard to
re-verify by eye (PID loops, kinematics, calibration).

One piece of dead code was dropped rather than ported: a `NeoPixelConnect` status-LED
object was instantiated in `robot.cpp` but never actually used anywhere. If you want
status-LED support back, drive the WS2812 directly with the PIO program from
`pico-examples/pio/ws2812` and wire it into `set_led_color()` in `main.cpp`.

**Calibration data (`LittleFS`) and single-core flash writes.** pico-vfs's flash
block device already wraps its erase/program operations in pico-sdk's
`flash_safe_execute()`, which is the mechanism that keeps flash writes safe with
respect to code executing (from flash) on the other core. Per your note, all flash
writes in this project happen from a single core, which is the simpler and
lower-risk case; you shouldn't need `PICO_COPY_TO_RAM` for that. If you ever do move
calibration saves to run from both cores, re-check pico-vfs's
[`LIMITATION.md`](https://github.com/oyama/pico-vfs/blob/main/LIMITATION.md#file-system-limitations)
first.

**Overclocking, hardware timers, multicore spinlocks, SPI encoders, and the PIO-based
motor driver are all unchanged** -- they were already plain pico-sdk and didn't need
touching.

## Verified

This exact tree (with the three submodules from `lib/README.md` checked out) was
built end-to-end against a real `arm-none-eabi-gcc` + `cmake`/`ninja` toolchain while
this port was put together, and compiled and linked cleanly into a `.uf2`. A handful
of real issues surfaced only by actually compiling it were fixed along the way:
missing `hardware_pio`/`hardware_dma` link libraries, a couple of Arduino-only macros
(`PI`, `DEG_TO_RAD`) with no pico-sdk equivalent, and a couple of missing standard
includes (`hardware/sync.h`, `<cstdio>`) that arduino-pico used to pull in for free
via `Arduino.h`.

## Suggested next steps

- Build it yourself against your actual hardware and confirm the servo loop's
  achieved update rate matches what you had before (it should, given the
  `taskYIELD()`-not-`vTaskDelay()` choice above, but worth confirming on-device).
- If you add more tasks later (telemetry, a watchdog, etc.), give them explicit
  priorities/affinities in `main.cpp` alongside `command_task`/`servo_task`.
- `configTOTAL_HEAP_SIZE` in `FreeRTOSConfig.h` is set to 128KB and
  `PICO_FS_DEFAULT_SIZE`/the `pico_enable_filesystem(... SIZE ...)` call in
  `src/CMakeLists.txt` reserves 512KB of flash for the littlefs volume -- both are
  reasonable starting points, tune to taste.
