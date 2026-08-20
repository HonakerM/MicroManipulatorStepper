# Third-party dependencies

This project depends on three external repos, referenced by CMakeLists.txt as
`lib/pico-sdk`, `lib/FreeRTOS-Kernel`, and `lib/pico-vfs`. They are **not** vendored
into this repo (they're large) -- set them up with one of the two options below.

## Option A: git submodules (recommended)

A ready-to-use `.gitmodules` file is included at the project root. From the project
root:

```bash
git submodule add -b master https://github.com/raspberrypi/pico-sdk.git lib/pico-sdk
git -C lib/pico-sdk submodule update --init   # pico-sdk's own submodules (tinyusb, etc.)

git submodule add https://github.com/raspberrypi/FreeRTOS-Kernel.git lib/FreeRTOS-Kernel

git submodule add https://github.com/oyama/pico-vfs.git lib/pico-vfs
git -C lib/pico-vfs submodule update --init   # pulls in littlefs
```

(If you're starting from a fresh clone of *this* repo and `.gitmodules` is already
present, just run `git submodule update --init --recursive` instead of the `add`
commands above.)

## Option B: plain clones (if you don't want submodules)

```bash
git clone https://github.com/raspberrypi/pico-sdk.git lib/pico-sdk
git -C lib/pico-sdk submodule update --init

git clone https://github.com/raspberrypi/FreeRTOS-Kernel.git lib/FreeRTOS-Kernel

git clone https://github.com/oyama/pico-vfs.git lib/pico-vfs
git -C lib/pico-vfs submodule update --init
```

## Why these specific repos

- **pico-sdk** -- standard, no surprises.
- **FreeRTOS-Kernel -- must be `raspberrypi/FreeRTOS-Kernel`, not the upstream
  `FreeRTOS/FreeRTOS-Kernel`.** Only the Raspberry Pi fork carries the
  `portable/ThirdParty/GCC/RP2350_ARM_NTZ` port, which is the dual-core SMP FreeRTOS
  port for the RP2350 (Pico 2) that this project's `FreeRTOS_Kernel_import.cmake`
  (copied from that same fork) looks for. The upstream repo only has an RP2040 port.
- **pico-vfs** (`oyama/pico-vfs`) -- provides the POSIX file API (`fopen`/`opendir`/
  `stat`/...) that `src/compat/LittleFS_compat.*` is built on top of, backed by
  littlefs on the onboard flash. This is what replaces arduino-pico's `LittleFS`
  library.

## Building

Once the three dependencies above are in place:

```bash
mkdir build && cd build
cmake -G Ninja ..
ninja
```

This produces `src/MotionControllerRP.uf2` (plus `.elf`/`.bin`), which you flash the
same way as any other pico-sdk UF2: hold BOOTSEL, plug in, drag the file onto the
mounted drive.

If `cmake` can't find the SDK/kernel automatically, pass the paths explicitly:

```bash
cmake -G Ninja -DPICO_SDK_PATH=../lib/pico-sdk -DFREERTOS_KERNEL_PATH=../lib/FreeRTOS-Kernel ..
```

This exact combination (these three repos, this CMakeLists.txt, this
FreeRTOSConfig.h) was test-built end to end against the real `arm-none-eabi-gcc`
toolchain while this port was being put together, and compiled/linked cleanly.
