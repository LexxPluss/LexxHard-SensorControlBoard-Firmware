# LexxPluss Sensor Control Board Software

[![CI](https://github.com/LexxPluss/LexxHard-SensorControlBoard-Firmware/actions/workflows/main.yml/badge.svg)](https://github.com/LexxPluss/LexxHard-SensorControlBoard-Firmware/actions/workflows/main.yml)
[![release](https://github.com/LexxPluss/LexxHard-SensorControlBoard-Firmware/actions/workflows/release.yml/badge.svg)](https://github.com/LexxPluss/LexxHard-SensorControlBoard-Firmware/actions/workflows/release.yml)

## For Docker

## Install dependencies

```bash
$ git clone https://github.com/LexxPluss/LexxHard-SensorControlBoard-Firmware
$ cd LexxHard-SensorControlBoard-Firmware
$ make build
```
## Setup Zephyr

```bash
$ make setup
$ make update
```
## Build
### Build bootloader (MCUboot)

```bash
$ make bootloader
```
### Build firmware

```bash
$ make firmware
```
```bash
$ cp ./build/zephyr/zephyr.signed.confirmed.bin LexxHard-SensorControlBoard-Firmware-Update-?.?.?.bin

$ dd if=/dev/zero bs=1k count=256 | tr "\000" "\377" > bl_with_ff.bin
$ dd if=build-mcuboot/zephyr/zephyr.bin of=bl_with_ff.bin conv=notrunc
$ cat bl_with_ff.bin build/zephyr/zephyr.signed.bin >  LexxHard-SensorControlBoard-Firmware-Initial-?.?.?.bin
```

The default build includes the ToF feature -- L4 cliff, the L7 hanging-object grid and the Package
sensor -- because [`features_dasher.cmake`](lexxpluss_apps/cmake/features_dasher.cmake), the
Makefile's default `FEATURES_FILE`, includes the Package configuration described below. So
`make firmware`, `make firmware_two_state_ksw` and `make firmware_interlock` produce ToF-enabled
images, and a release tag publishes one. Which machine an image goes to is still a fitment
question: a missing Package sensor only raises a diagnostic fault that nothing consumes (measured
on dasher2), and what a machine without the cliff or L7 sensors does on the consumer side is not
established here.

### Build firmware ( enable interlock )

```bash
$ make firmware_interlock
```

### Build firmware ( ToF release configuration )

The ToF configuration (L4 cliff, L7 hanging-object grid and host commissioning) is the ordinary
firmware built with a feature file, not a target of its own. **This selection is chain-only**; the
Package configuration below adds the Package sensor, so name this one when the Package runtime has
to be out of the image:

```bash
$ make clean
$ make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_tof_release.cmake
```

`make clean` comes first because `build/` is shared: a stale CMake cache would carry one feature
selection into the next image. Clean again before building anything else, and move whatever is in
`out/` aside first -- this image writes the same `out/zephyr.signed.bin`,
`out/zephyr.signed.confirmed.bin` and `out/firmware.bin` as the standard one.

**A tag does not build this image.** The release workflow is unchanged, so a release publishes the
standard assets only; build this one by hand when it is needed.

**Flashing it does not install the L7 driver blob.** The first 256 KiB of the Initial image are the
bootloader partition and `storage_partition`, and the storage half is `0xff`; the blob lives in
`storage_partition`, so an erased board has none. Provision the blob after flashing. Until that is
done the L7 paths stay silent, however complete the rest of the chain looks.

#### Test-boot image for CAN DFU

`make dfu_test_image` derives `out/zephyr.test.bin` from `out/zephyr.signed.confirmed.bin` by
resetting the trailer's `image_ok` byte to `0xff`, which is what makes CAN DFU request a slot swap.
It then checks that the two files are still a pair: both padded to the slot, differing in exactly
that one byte, `01` against `ff`.

It is a bench tool. **It is not uploaded with a Release**, and **a test boot is not rollback**: the
application self-confirms before the subsystem threads start, so an image that boots far enough to
do that will not revert on a later runtime failure.

### Package ToF release configuration

The Package configuration is the ToF release plus the Package sensor -- one VL53L7CX on `i2c4` @0x29
behind J22, publishing a grid and a health frame on CAN at 5 Hz. **This is what plain
`make firmware` builds**: the default feature file includes
[`features_package_tof_release.cmake`](lexxpluss_apps/cmake/features_package_tof_release.cmake), so
from the command line and CI the commands are `make clean` and `make firmware VERSION=<version>`,
with no `FEATURES_FILE` named, and the assets are the standard ones from the section above.

**It carries its own flash fit, and the margin is thin.** The chain and the Package runtime together
overflow the 256 KiB slot at link time on the production base, so
[`tof_package_flashfit.conf`](lexxpluss_apps/overlays/tof_package_flashfit.conf) turns the shell
conveniences and SPI DMA off -- the shell commands themselves stay -- for every configuration that
takes it. With it the image signs at 258,408 B, leaving 3,304 B of the 261,712 B ceiling, which is
1.26%: one new log string or one re-enabled shell convenience breaks the link again. Those figures
were measured at f967fb0b; the selection has not changed since, but re-measure on the commit a
release is actually cut from. The fragment grew past the tree-wide flash reductions when the LCS
host arrived, so landing those no longer retires it; the next real headroom is picolibc.

**The bench deadman is off, as release policy** -- a resident image carries no unattended timed
revert, which is what keeps it confirmed. The watchdog is untouched and still judges the L7 path,
and it now judges the chain and Package populations from separate clocks, so progress by a healthy
one no longer refreshes the judgement for a wedged one. What no run has shown is the two populations
declaring a long operation at the same time:
[`features_package_tof_release.cmake`](lexxpluss_apps/cmake/features_package_tof_release.cmake)
states why the ordering keeps them apart, and why that is an argument rather than a measurement.

**Flashing it does not install the L7 driver blob**, for the reason the ToF section gives: provision
the blob afterward, or the Package path stays silent whatever else looks healthy.

---
## Program of the built firmware

### First time program

Program the bootloader and signed firmware after erasing the entire Flash ROM.

```bash
$ brew install stlink
$ st-flash --reset --connect-under-reset erase
$ st-flash --reset --connect-under-reset write build-mcuboot/zephyr/zephyr.bin 0x8000000
$ st-flash --reset --connect-under-reset write build/zephyr/zephyr.signed.bin 0x8040000
```

### Update

Program the firmware for update to the update area.

```bash
$ st-flash --reset --connect-under-reset write build/zephyr/zephyr.signed.confirmed.bin 0x8080000
```

## Program of the released firmware

```bash
$ st-flash --reset --connect-under-reset erase
$ st-flash --reset --connect-under-reset write LexxHard-SensorControlBoard-Firmware-Initial-v?.?.? 0x8000000
```

## Update via ROS

(There is no updater for SensorControlBoard yet)

## License

Copyright (c) 2022, LexxPluss Inc. Released under the [BSD License](LICENSE).
