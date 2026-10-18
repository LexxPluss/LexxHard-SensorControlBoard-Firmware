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

### ToF assets

A release that carries the ToF configuration (L4 cliff, L7 hanging-object grid and host
commissioning) adds three files. They are separate from the standard assets above because they are
a different image, not a different way of delivering the same one.

- `...-ToF-Initial-v?.?.?.bin` — bootloader and application in one file, written from `0x8000000`
  after an erase, exactly like the standard Initial image.
- `...-ToF-Update-v?.?.?.bin` — the padded, confirmed image for an ordinary update.
- `...-ToF-DFU-Test-v?.?.?.bin` — the same image with `image_ok` left at `0xff`, for a test boot
  over the CAN DFU. The board reverts to the previous firmware on the next reset unless the
  application confirms itself.

`SHA256SUMS-v?.?.?.txt` lists every asset in the release.

**Flashing the ToF Initial image does not install the L7 driver blob.** Its first 256 KiB are the
bootloader partition and `storage_partition`, and the storage half is `0xff`; the blob lives in
`storage_partition`, so an erased board has none. Provision the blob after flashing. Until that is
done the L7 paths stay silent, however complete the rest of the chain looks.

## Update via ROS

(There is no updater for SensorControlBoard yet)

## License

Copyright (c) 2022, LexxPluss Inc. Released under the [BSD License](LICENSE).
