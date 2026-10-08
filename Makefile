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
# Absolute path to extra/ (custom -DBOARD_ROOT / -DZEPHYR_EXTRA_MODULES root).
# Defaults to /workdir, matching the "volumes: .:/workdir" mount in docker-compose.yml.
# When IN_HOST=1 (no container), override with the worktree's absolute path, e.g.
# make IN_HOST=1 WORKDIR=$PWD firmware
WORKDIR:=$(if $(WORKDIR),$(WORKDIR),/workdir)
RUNNER:=$(if $(IN_HOST),$(),docker compose run --rm zephyrbuilder)

.PHONY: all
all: bootloader firmware

.PHONY: clean
clean:
	rm -rf build-mcuboot build build-bypass-safety-lidar build-test-tof-packer \
	        build-test-tof-cliff-packer build-test-tof-mapping-authority \
	        build-test-tof-commissioning build-test-tof-tail-isolation build-test-tof-mapping-proof \
	        build-tof-cliff twister-out* build-test-tof-cliff-sensor build-test-tof-uld-status \
	        build-test-tof-enumerator build-test-tof-auto-commission build-tof-chain \
	        build-tof-l7 build-test-tof-l7-port build-test-tof-l7-sensor \
	        build-test-tof-l7-blob build-test-tof-l7-uld-stop

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

.PHONY: test
test: check_language_boundary
	$(RUNNER) west zephyr-export
	$(RUNNER) west twister -T lexxpluss_apps/tests --platform native_sim -v -A ${WORKDIR}/extra

.PHONY: bootloader
bootloader:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb bootloader/mcuboot/boot/zephyr -d build-mcuboot -- -DBOARD_ROOT=${WORKDIR}/extra
	mv build-mcuboot/zephyr/zephyr.bin out/zephyr.bin

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
# The only suite that compiles a vendor translation unit, and it compiles the PATCHED
# copy: it pins that a failed VL53LX_get_device_results() is reported as a failure and
# does not move the device's stream-count history. Dropping the patch fails it.
.PHONY: test_tof_uld_status
test_tof_uld_status:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_uld_status -d build-test-tof-uld-status -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

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

# Host-side tests for the prove-then-start sequencer: every path that must NOT reach start(), the
# bounded retry, and the hooks that refuse before anything is attempted. No device, no bus, no proof.
.PHONY: test_tof_auto_commission
test_tof_auto_commission:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_auto_commission -d build-test-tof-auto-commission -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

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

# Host-side tests for the hanging-object (VL53L7CX) driver and the stored device-firmware blob.
# Three suites, separate because they fail for different reasons.

# The Zephyr port, at wire level, with the vendor ULD deliberately absent: it pins our 16-bit index,
# repeated start, the 328-byte segmentation bound and the sticky errno contract, without freezing
# ST's internal register sequence into this repository.
.PHONY: test_tof_l7_port
test_tof_l7_port:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_port -d build-test-tof-l7-port -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# The adapter, against fake ULD and runtime symbols. There is no test-only open path: the object
# assignment still calls tof_l7_runtime::firmware_data(), and the link substitute only controls the
# answer it gets.
.PHONY: test_tof_l7_sensor
test_tof_l7_sensor:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_sensor -d build-test-tof-l7-sensor -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# The stored blob record, with the real SHA-256 and the real CRC against the GENERATOR's own output,
# committed as the suite's golden record. A layout written once at manufacture and read at every
# boot needs its two implementations pinned to each other.
#
# TWO GENERATOR CHECKS RUN FIRST, and they pin different things. The golden record proves the
# committed fixture is still what the generator emits, so the suite cannot drift from the writer.
# The expectation check proves the accept-list compiled INTO the image still describes the payload
# in the vendored ULD -- a re-vendored snapshot with a different device firmware would otherwise
# leave the two disagreeing, and the first thing to notice would be a board refusing to range.
.PHONY: test_tof_l7_blob
test_tof_l7_blob:
	$(RUNNER) bash -c 'python3 docs/can/gen_l7_blob_record.py golden --out /tmp/golden_record.h && diff -u lexxpluss_apps/tests/tof_l7_blob/src/golden_record.h /tmp/golden_record.h'
	$(RUNNER) python3 docs/can/gen_l7_blob_record.py pack lexxpluss_apps/third_party/st/vl53l7cx_uld/upstream/modules/vl53l7cx_buffers.h --c-array VL53L7CX_FIRMWARE --expect-header lexxpluss_apps/third_party/st/vl53l7cx_uld/zephyr/vl53l7cx_blob_expectation.hpp --check
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_blob -d build-test-tof-l7-blob -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# vl53l7cx_stop_ranging() against a bus the test controls, with the REAL vendor function compiled
# from the patched copy. The adapter suite substitutes the whole ULD, which is right for the adapter
# and is exactly why it cannot see a defect inside the vendor function itself: on timeout the
# upstream code folded in the last polled byte, which is zero precisely when the stop was not
# confirmed, so a five-second wait returned OK. See zephyr/patches/0002.
.PHONY: test_tof_l7_uld_stop
test_tof_l7_uld_stop:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_uld_stop -d build-test-tof-l7-uld-stop -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

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
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DBOARD_ROOT=${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr.signed.confirmed.bin

.PHONY: firmware_two_state_ksw
firmware_two_state_ksw:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DUSE_TWO_STATE_KEY_SWITCH=1 -DBOARD_ROOT=${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=${WORKDIR}/extra -DVERSION=${VERSION}
	mv build/zephyr/zephyr.signed.bin out/zephyr_two_state_ksw.signed.bin
	mv build/zephyr/zephyr.signed.confirmed.bin out/zephyr_two_state_ksw.signed.confirmed.bin

.PHONY: firmware_interlock
firmware_interlock:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -b lexxpluss_scb lexxpluss_apps -- -DENABLE_INTERLOCK=1 -DBOARD_ROOT=${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=${WORKDIR}/extra -DVERSION=${VERSION}
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
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-bypass-safety-lidar -- -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 -DBOARD_ROOT=${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-bypass-safety-lidar/zephyr/zephyr.signed.bin out/zephyr_bypass_safety_lidar.signed.bin
	mv build-bypass-safety-lidar/zephyr/zephyr.signed.confirmed.bin out/zephyr_bypass_safety_lidar.signed.confirmed.bin

# ToF chain enumeration build (AMRSW-2322 Phase 2): production firmware plus
# the chain glue and the manual `tof enum` commissioning command. Requires
# the NACK-classification patch (verified first) and stacks on the Dasher
# safety-lidar bypass like the diagnostic build. Dedicated build directory
# for the usual cache-leak reason.
# The on-machine cliff build: the chain PLUS the L4 cliff ULD, acquisition, packer, publisher and
# CAN glue. Distinct from firmware_tof_chain, which is the chain only, with no cliff data path.
# This is the single cliff capacity number now: the staged TOF_CLIFF_BUDGET probe was retired in
# the same commit that made this path reachable, because its per-step storage double-counted
# against the production storage and its increments no longer isolated anything.
#
# Delivered as a padded TEST image, like firmware_tof_chain and for the same measured reason: the
# CAN DFU writes raw bytes into slot1 and never calls boot_request_upgrade, so only a trailer
# embedded in the file can request a swap. An unpadded signed.bin therefore sits in slot1 doing
# nothing while the machine keeps running the old firmware -- and that looks identical to a revert.
# main.cpp confirms the image after thread creation, so a crash in main initialisation (which is
# where the cliff bootstrap runs) rolls back on the next boot.
#
# This image produces the 0x217 health heartbeat and does NOT produce measurement frames, because
# the PROVEN clamp is applied unconditionally at the single authorisation exit.
#
# That is the only thing the clamp decides. It does not decide the proof: `tof cliff prove` succeeds
# or fails on its evidence, and the four cliff roles are no longer unknown -- they are frozen from
# the assembly connectivity drawing in tof_chain_spec.hpp, which is what makes the production spec
# provable at all. What is still open is the hardware: walk 1 has never reached COMPLETE at the
# 400 kHz this overlay pins, so a run on a real machine fails there rather than at the role table.
#
# NO SAFETY-LIDAR BYPASS, unlike firmware_tof_chain, which this target was first copied from. That
# flag belongs to a bench image and this one is meant to be a product build; carrying it by
# inheritance is how a bypass ships. firmware_bypass_safety_lidar remains the named target for a
# machine with no safety lidar fitted, and a bench that needs both cliff and the bypass needs its
# own target rather than this one quietly being both.
#
# firmware_tof_chain still carries the flag. That is pre-existing and deliberately left alone here;
# whether a bring-up target should keep it is a separate decision from what this one ships with.

# The cliff image plus the hanging-object ULD. BRING-UP, NOT A PRODUCT BUILD, and it is named here
# rather than left to a command line so that what it carries is reviewable.
#
# NO SAFETY-LIDAR BYPASS, for the reason firmware_tof_cliff gives below: the development images this
# driver comes from carried one, and inheriting it is how a bypass ships.
#
# IT FITS TODAY, AND THAT IS NOT A PASS. Measured at 245,504 B signed against the 261,712 B ceiling
# with the filesystem stack still configured -- 72 B more than the same image without the L7 flag,
# because nothing calls the driver and --gc-sections drops it. The development images this comes
# from did not fit and dropped the filesystem; they also carried the publisher, the monitoring and
# the recovery. Binding the grid operations adds the 11,531 B those objects compile to, before any
# of the rest. So this target is for compiling and measuring the driver, the filesystem decision is
# lexxpluss_apps/CMakeLists.txt's to explain and nobody's to inherit, and whoever revisits it must
# re-measure on the configuration they are shipping.
.PHONY: firmware_tof_l7
firmware_tof_l7:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-tof-l7 -- -DENABLE_TOF_CHAIN=1 -DENABLE_TOF_CLIFF_ULD=ON -DENABLE_TOF_L7_ULD=ON -DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}

.PHONY: firmware_tof_cliff
firmware_tof_cliff:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-tof-cliff -- -DENABLE_TOF_CHAIN=1 -DENABLE_TOF_CLIFF_ULD=ON -DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-tof-cliff/zephyr/zephyr.signed.bin out/zephyr_tof_cliff.signed.bin
	mv build-tof-cliff/zephyr/zephyr.signed.confirmed.bin out/zephyr_tof_cliff.signed.confirmed.bin
	cp out/zephyr_tof_cliff.signed.confirmed.bin out/zephyr_tof_cliff.test.bin
	printf '\377' | dd of=out/zephyr_tof_cliff.test.bin bs=1 seek=$$(($$(stat -c%s out/zephyr_tof_cliff.test.bin) - 24)) conv=notrunc status=none

#
# The `tof enum` command is present but is NOT expected to complete on this
# image: overlays/tof_chain.overlay pins the bus at 400 kHz for the acquisition
# schedule, and commissioning was measured at 0/69 complete walks there. The
# overlay comment carries the measurement and names the fix, which is not in
# this change.
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

