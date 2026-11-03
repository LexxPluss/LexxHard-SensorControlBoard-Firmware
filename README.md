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

### Build firmware ( enable interlock )

```bash
$ make firmware_interlock
```

### Build firmware ( ToF release configuration )

The ToF configuration (L4 cliff, L7 hanging-object grid and host commissioning) is the ordinary
firmware built with a feature file, not a target of its own:

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

### Build firmware ( Package ToF release configuration )

The Package configuration is the ToF release plus the Package sensor -- one VL53L7CX on `i2c4` @0x29
behind J22, publishing a grid and a health frame on CAN at 5 Hz -- built through the same entry with
a different feature file:

```bash
$ make clean
$ make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_package_tof_release.cmake
```

Asset names, the shared `build/` cache and the `make clean` rule are exactly as in the section
above, and a tag does not build this image either.

**It does not link on a tree without the flash reductions.** The chain and the Package runtime
together overflow the 256 KiB slot by about 3.5 KB, so this configuration requires the picolibc,
shell trim, SPI DMA and SD/filesystem changes to be present. With them the image signs at
234,368 B, with 27,344 B spare.

**The unattended revert is off, as release policy** -- a resident image carries no unattended reset,
so nothing here puts the board back if the Package thread wedges.
[`features_package_tof_release.cmake`](lexxpluss_apps/cmake/features_package_tof_release.cmake)
carries that and the one open question this image raises: both sensor populations report L7 progress
into a single watchdog slot, so a healthy one can mask a wedged one until that is addressed.

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
