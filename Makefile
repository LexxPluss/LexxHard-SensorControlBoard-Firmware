# Copyright (c) 2024, LexxPluss Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

VERSION:=$(shell git describe --tags HEAD | cut -c2-)
WORKDIR:=$(if $(WORKDIR),$(),workdir)
RUNNER:=$(if $(IN_HOST),$(),docker compose run --rm zephyrbuilder)

.PHONY: all
all: bootloader firmware

.PHONY: clean
clean:
	rm -rf build-mcuboot build

.PHONY: distclean
distclean: clean
	rm -rf build-mcuboot build bootloader modules tools zephyr out .west

.PHONY: build
build: docker-compose.yml Dockerfile
	docker compose build

.PHONY: setup
setup:
	$(RUNNER) west init -l lexxpluss_apps
	$(RUNNER) west update
	$(RUNNER) west config --global zephyr.base-prefer configfile
	mkdir -p out

.PHONY: update
update:
	$(RUNNER) west update

.PHONY: bootloader
bootloader:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb bootloader/mcuboot/boot/zephyr -d build-mcuboot -- -DBOARD_ROOT=/${WORKDIR}/extra
	mv build-mcuboot/zephyr/zephyr.bin out/zephyr.bin

.PHONY: firmware
firmware:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr.signed.confirmed.bin

.PHONY: firmware_two_state_ksw
firmware_two_state_ksw:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DUSE_TWO_STATE_KEY_SWITCH=1 -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr_two_state_ksw.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr_two_state_ksw.signed.confirmed.bin

.PHONY: firmware_interlock
firmware_interlock:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DENABLE_INTERLOCK=1 -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr_interlock.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr_interlock.signed.confirmed.bin

.PHONY: firmware_initial
firmware_initial:
	$(MAKE) bootloader
	$(MAKE) firmware
	dd if=/dev/zero bs=1k count=256 | tr "\000" "\377" > out/bl_with_ff.bin
	dd if=out/zephyr.bin of=out/bl_with_ff.bin conv=notrunc
	cat out/bl_with_ff.bin out/zephyr.signed.bin > out/firmware.bin

.PHONY: firmware_two_state_ksw_initial
firmware_two_state_ksw_initial:
	$(MAKE) bootloader
	$(MAKE) firmware_two_state_ksw
	dd if=/dev/zero bs=1k count=256 | tr "\000" "\377" > out/bl_with_ff.bin
	dd if=out/zephyr.bin of=out/bl_with_ff.bin conv=notrunc
	cat out/bl_with_ff.bin out/zephyr_two_state_ksw.signed.bin > out/firmware_two_state_ksw.bin

.PHONY: firmware_interlock_initial
firmware_interlock_initial:
	$(MAKE) bootloader
	$(MAKE) firmware_interlock
	dd if=/dev/zero bs=1k count=256 | tr "\000" "\377" > out/bl_with_ff.bin
	dd if=out/zephyr.bin of=out/bl_with_ff.bin conv=notrunc
	cat out/bl_with_ff.bin out/zephyr_interlock.signed.bin > out/firmware_interlock.bin

