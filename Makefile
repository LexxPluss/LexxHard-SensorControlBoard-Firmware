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
	rm -rf build-mcuboot build build-bypass-safety-lidar build-test-tof-packer build-test-tof-cliff-packer build-test-tof-mapping-authority build-test-tof-commissioning build-test-tof-tail-isolation build-test-tof-mapping-proof build-tof-cliff

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
	./scripts/manage_zephyr_patches.sh apply
	mkdir -p out

.PHONY: update
update:
	./scripts/manage_zephyr_patches.sh unapply
	$(RUNNER) west update
	./scripts/manage_zephyr_patches.sh apply

.PHONY: bootloader
bootloader:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb bootloader/mcuboot/boot/zephyr -d build-mcuboot -- -DBOARD_ROOT=/${WORKDIR}/extra
	mv build-mcuboot/zephyr/zephyr.bin out/zephyr.bin

.PHONY: test
test: check_language_boundary
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b native_sim lexxpluss_apps/tests/shutter_limit_switch -d build-test -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the ToF grid packer, driven by the golden vectors in
# docs/can/ and pinning the contract SHA-256 (the firmware half of the
# cross-repository lock; SCBDriver pins the same SHA on the decoder side).
# The generator check runs first: it fails loudly if the contract was edited
# without regenerating the vectors, which the SHA pins alone cannot see.
.PHONY: test_tof_packer
test_tof_packer:
	$(RUNNER) python3 docs/can/gen_golden_vectors.py --check
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_packer -d build-test-tof-packer -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the cliff measurement packer: the contract SHA pin (the firmware
# half of the cross-repository lock) and the normative reduction, which the layout
# vectors cannot cover because the pre-reduction target list never reaches the wire.
# The generator check runs first, for the same reason the grid target does it.
.PHONY: test_tof_cliff_packer
test_tof_cliff_packer:
	$(RUNNER) python3 docs/can/gen_cliff_golden_vectors.py --check
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_cliff_packer -d build-test-tof-cliff-packer -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the cliff (VL53L4CX) sensor layer. Two levels in one image:
# the port's wire shape and errno path through an emulated I2C controller, and the
# read_once adapter against fakes that reproduce the ULD's own defects. The ULD's
# sources are deliberately absent from this build -- driving the real ULD would mean
# freezing a vendor-internal register sequence into our tests.
.PHONY: test_tof_cliff_sensor
test_tof_cliff_sensor:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_cliff_sensor -d build-test-tof-cliff-sensor -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the ToF enumeration layer: currently the production
# guarded readdress (exact-traffic properties the enumerator fakes cannot
# prove, above all zero-writes-after-transport-error); the enumeration state
# machine suite joins here.
.PHONY: test_tof_enumerator
test_tof_enumerator:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_enumerator -d build-test-tof-enumerator -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the mapping authority: every way a challenge, an epoch or a proof can fail to authorise, and the guarantee that a refusal costs nothing.
.PHONY: test_tof_mapping_authority
test_tof_mapping_authority:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_mapping_authority -d build-test-tof-mapping-authority -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the commissioning orchestrator: the transaction that quiesces
# acquisition, runs two walks plus tail isolation, and asks the authority to commit.
# 13 use an injected fake quiesce; 3 link the real acquisition layer.
.PHONY: test_tof_commissioning
test_tof_commissioning:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commissioning -d build-test-tof-commissioning -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for tail isolation: the sequence that proves the tail answers and its neighbour is silent, without destroying the evidence it just gathered.
.PHONY: test_tof_tail_isolation
test_tof_tail_isolation:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_tail_isolation -d build-test-tof-tail-isolation -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the mapping proof: the two-walk fingerprint comparison and every refusal it can return, against a fake chain. No device, no bus, no ULD.
.PHONY: test_tof_mapping_proof
test_tof_mapping_proof:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_mapping_proof -d build-test-tof-mapping-proof -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# The golden-vector generators are Python and live in docs/can/ as offline tooling, so
# they do enter the production Git branch. This gate is what keeps that from becoming
# Python in the product: it fails if any .py appears outside docs/can/, or if any build
# description or application file references one. Runs on the host, needs only git.
.PHONY: check_language_boundary
check_language_boundary:
	./scripts/check_language_boundary.sh

.PHONY: firmware
firmware:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr.signed.confirmed.bin

.PHONY: firmware_two_state_ksw
firmware_two_state_ksw:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DUSE_TWO_STATE_KEY_SWITCH=1 -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr_two_state_ksw.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr_two_state_ksw.signed.confirmed.bin

.PHONY: firmware_interlock
firmware_interlock:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DENABLE_INTERLOCK=1 -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr_interlock.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr_interlock.signed.confirmed.bin

# Diagnostic-only target: bypass ONLY the safety-lidar assertion; KEEP E-stop active.
# For Dasher (no safety-lidar hardware) actuator direct-drive testing -- the real
# E-stop still gates the actuator. Robot must NOT be allowed to drive while running this.
# Uses a dedicated build directory (build-bypass-safety-lidar) so the bypass CMake cache
# variable can never leak into a later `make firmware` that reuses build/ and would
# silently emit a bypassed binary under the production filename out/zephyr.signed.bin.
.PHONY: firmware_bypass_safety_lidar
firmware_bypass_safety_lidar:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-bypass-safety-lidar -- -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-bypass-safety-lidar/zephyr/zephyr.signed.bin out/zephyr_bypass_safety_lidar.signed.bin
	mv build-bypass-safety-lidar/zephyr/zephyr.signed.confirmed.bin out/zephyr_bypass_safety_lidar.signed.confirmed.bin

# ToF chain enumeration build (AMRSW-2322 Phase 2): production firmware plus
# the chain glue and the manual `tof enum` commissioning command. Requires
# the NACK-classification patch (verified first) and stacks on the Dasher
# safety-lidar bypass like the diagnostic build. Dedicated build directory
# for the usual cache-leak reason.
# The on-machine cliff build: the chain PLUS the L4 cliff ULD, acquisition, packer, publisher and
# CAN glue. Distinct from firmware_tof_chain (chain only, no cliff data path) and from the
# TOF_CLIFF_BUDGET points (those link a measurement probe that drives one cycle itself and must
# never reach a robot).
#
# Delivered as a padded TEST image, like firmware_tof_chain and for the same measured reason: the
# CAN DFU writes raw bytes into slot1 and never calls boot_request_upgrade, so only a trailer
# embedded in the file can request a swap. An unpadded signed.bin therefore sits in slot1 doing
# nothing while the machine keeps running the old firmware -- and that looks identical to a revert.
# main.cpp confirms the image after thread creation, so a crash in main initialisation (which is
# where the cliff bootstrap runs) rolls back on the next boot.
#
# PROVEN is still clamped and the four cliff roles are still unknown, so this image produces the
# 0x217 health heartbeat and refuses `tof cliff prove`. It does NOT produce measurement frames.
.PHONY: firmware_tof_cliff
firmware_tof_cliff:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-tof-cliff -- -DENABLE_TOF_CHAIN=1 -DENABLE_TOF_CLIFF_ULD=ON -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 -DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-tof-cliff/zephyr/zephyr.signed.bin out/zephyr_tof_cliff.signed.bin
	mv build-tof-cliff/zephyr/zephyr.signed.confirmed.bin out/zephyr_tof_cliff.signed.confirmed.bin
	cp out/zephyr_tof_cliff.signed.confirmed.bin out/zephyr_tof_cliff.test.bin
	printf '\377' | dd of=out/zephyr_tof_cliff.test.bin bs=1 seek=$$(($$(stat -c%s out/zephyr_tof_cliff.test.bin) - 24)) conv=notrunc status=none

.PHONY: firmware_tof_chain
firmware_tof_chain:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-tof-chain -- -DENABLE_TOF_CHAIN=1 -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 -DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-tof-chain/zephyr/zephyr.signed.bin out/zephyr_tof_chain.signed.bin
	mv build-tof-chain/zephyr/zephyr.signed.confirmed.bin out/zephyr_tof_chain.signed.confirmed.bin
	cp out/zephyr_tof_chain.signed.confirmed.bin out/zephyr_tof_chain.test.bin
	printf '\377' | dd of=out/zephyr_tof_chain.test.bin bs=1 seek=$$(($$(stat -c%s out/zephyr_tof_chain.test.bin) - 24)) conv=notrunc status=none

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

