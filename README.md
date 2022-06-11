# LexxPluss Main Board Software

[![CI](https://github.com/LexxPluss/LexxHard-MainBoard-Firmware/actions/workflows/main.yml/badge.svg)](https://github.com/LexxPluss/LexxHard-MainBoard-Firmware/actions/workflows/main.yml)
[![release](https://github.com/LexxPluss/LexxHard-MainBoard-Firmware/actions/workflows/release.yml/badge.svg)](https://github.com/LexxPluss/LexxHard-MainBoard-Firmware/actions/workflows/release.yml)

## Install dependencies

Prepare a development environment referring to
https://docs.zephyrproject.org/2.7.0/getting_started/

In macOS, it probably looks like this.

### Install Homebrew

```bash
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Install Utils

```bash
$ brew install cmake ninja gperf python3 ccache qemu dtc
$ pip3 install -U west
```

### Setup Zephyr

```bash
$ west init -l lexxpluss_apps
$ west update
$ west zephyr-export
$ pip3 install -r zephyr/scripts/requirements.txt
```

### Install Toolchain

Install Toolchain and set environment variables referring to
https://docs.zephyrproject.org/latest/getting_started/toolchain_3rd_party_x_compilers.html#gnu-arm-embedded

```bash
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=/Applications/ARM
```

## Build

### Build bootloader (MCUboot)

```bash
$ west build -b lexxpluss_mb01 bootloader/mcuboot/boot/zephyr -d build-mcuboot
```

### Build firmware

```bash
$ west build -p auto -b lexxpluss_mb01 lexxpluss_apps
```

## Program

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

Programming firmware for update via ROS will be supported soon.

## License

Copyright (c) 2022, LexxPluss Inc. Released under the [BSD License](https://github.com/LexxPluss/LexxHard-MainBoard-Firmware/blob/main/LICENSE).
