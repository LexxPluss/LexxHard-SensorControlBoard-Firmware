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
