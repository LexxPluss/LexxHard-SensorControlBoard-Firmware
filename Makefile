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
	rm -rf build-mcuboot build build-bypass-safety-lidar build-test-tof-packer

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

# The golden-vector generators are Python and live in docs/can/ as offline tooling, so
# they do enter the production Git branch. This gate is what keeps that from becoming
# Python in the product: it fails if any .py appears outside docs/can/, or if any build
# description or application file references one. Runs on the host, needs only git.
.PHONY: check_language_boundary
check_language_boundary:
	./scripts/check_language_boundary.sh

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

# Host-side tests for the mapping authority: the commit transaction, the epoch bitmap and the
# UNKNOWN/PROVEN/LOST/FAULT publication. begin_epoch is injected, which is the only way to
# test that a failed cycle reset leaves the state non-PROVEN.
.PHONY: test_tof_mapping_authority
test_tof_mapping_authority:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_mapping_authority -d build-test-tof-mapping-authority -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the commissioning orchestration: the order of the steps, recursive locking
# inside one chain session, which exit paths release the chain, and what is left open when a step
# fails. The chain mutex is a real k_mutex and the authority, proof, enumerator and isolation are the
# real components -- only the bus and the quiesce hook are faked, because faking any of the others
# would assume the answer.
# Host-side tests for the stored VL53L7CX device-firmware record. The bytes under test are the
# generator's own output (tests/tof_l7_blob/src/golden_record.h), which is what keeps
# the offline L7 blob-record generator and the C++ reader from drifting apart: the layout is written once at
# manufacture and read at every boot, and a disagreement has to fail in CI rather than on a board.
.PHONY: test_tof_l7_blob
test_tof_l7_blob:
	python3 docs/can/gen_l7_blob_record.py golden --out lexxpluss_apps/tests/tof_l7_blob/src/golden_record.h --check
	python3 docs/can/gen_l7_blob_record.py pack lexxpluss_apps/third_party/st/vl53l7cx_uld/upstream/modules/vl53l7cx_buffers.h --c-array VL53L7CX_FIRMWARE --expect-header lexxpluss_apps/third_party/st/vl53l7cx_uld/zephyr/vl53l7cx_blob_expectation.hpp --check
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_blob -d build-test-tof-l7-blob -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

.PHONY: test_tof_l7_port
test_tof_l7_port:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_port -d build-test-tof-l7-port -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

.PHONY: test_tof_l7_sensor
test_tof_l7_sensor:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_l7_sensor -d build-test-tof-l7-sensor -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

.PHONY: test_tof_commissioning
test_tof_commissioning:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commissioning -d build-test-tof-commissioning -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for tail isolation: the middle step of the proof transaction. The fake models the
# shift register and records every control operation, because the ORDER is the property -- an
# isolation that reaches the right end state through an all-off has destroyed the evidence it was
# sent to collect, and no end-state assertion can see the difference.
.PHONY: test_tof_tail_isolation
test_tof_tail_isolation:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_tail_isolation -d build-test-tof-tail-isolation -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the cliff mapping proof: the contract's walk/isolation/walk
# transaction, the semantic fingerprint and its L7 normalisation, and the one-shot
# challenge/token pair. Pure decision logic -- neither the enumerator's object file nor any
# driver is linked, because every arrangement worth testing is one a healthy machine cannot
# produce.
.PHONY: test_tof_mapping_proof
test_tof_mapping_proof:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_mapping_proof -d build-test-tof-mapping-proof -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the ToF enumeration layer: currently the production
# guarded readdress (exact-traffic properties the enumerator fakes cannot
# prove, above all zero-writes-after-transport-error); the enumeration state
# machine suite joins here.
.PHONY: test_tof_enumerator
test_tof_enumerator:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_enumerator -d build-test-tof-enumerator -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the commissioning protocol state machine: sessions, the validation order, the
# idempotency table and the per-boot budgets. The entropy source and the transaction are injected;
# the sequencer, codec and mapper are the real ones.
.PHONY: test_tof_commission_session
test_tof_commission_session:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commission_session -d build-test-tof-commission-session -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the BINDING, over native_sim's loopback CAN controller -- a real Zephyr CAN
# device, so the filter, the send path and the receive callback under test are the real ones. The
# proof, the acquisition start and the entropy draw are supplied by the test, because all three are
# hardware; the CAN path is not among them.
.PHONY: test_tof_commission_bind
test_tof_commission_bind:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commission_bind -d build-test-tof-commission-bind -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the runtime adapter: which identifier a frame is answered on, what a frame
# under another one does, when the session frame goes out, and that there is one worker. The
# identifiers in that suite are TEST values, not the allocated pair -- a suite using the real ones
# could not tell a runtime that reads its configuration from one that ignores it.
.PHONY: test_tof_commission_runtime
test_tof_commission_runtime:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commission_runtime -d build-test-tof-commission-runtime -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the commissioning downlink codec and the internal-to-wire mapper: byte-exact
# golden vectors, malformed frames, and every enumerator of all four mapped enums. Symbolic
# identifiers only -- nothing here is wired to a CAN filter.
.PHONY: test_tof_commission_wire
test_tof_commission_wire:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_commission_wire -d build-test-tof-commission-wire -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

.PHONY: test_tof_i2c_speed
test_tof_i2c_speed:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_i2c_speed -d build-test-tof-i2c-speed -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

# Host-side tests for the unattended prove-then-start sequence: default off, bounded retries, and
# start() reachable only from a proof that succeeded. Every hook is injected -- the proof is the
# existing transaction and is not reimplemented here.
.PHONY: test_tof_auto_commission
test_tof_auto_commission:
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b native_sim lexxpluss_apps/tests/tof_auto_commission -d build-test-tof-auto-commission -t run -- -DBOARD_ROOT=/${WORKDIR}/extra

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
.PHONY: firmware_tof_chain
firmware_tof_chain:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-tof-chain -- -DENABLE_TOF_CHAIN=1 -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 -DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-tof-chain/zephyr/zephyr.signed.bin out/zephyr_tof_chain.signed.bin
	mv build-tof-chain/zephyr/zephyr.signed.confirmed.bin out/zephyr_tof_chain.signed.confirmed.bin
	cp out/zephyr_tof_chain.signed.confirmed.bin out/zephyr_tof_chain.test.bin
	printf '\377' | dd of=out/zephyr_tof_chain.test.bin bs=1 seek=$$(($$(stat -c%s out/zephyr_tof_chain.test.bin) - 24)) conv=notrunc status=none

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

# The automatic-commissioning build: the cliff image PLUS the hardware RNG the session token needs.
# Its own flag, its own devicetree overlay and its own Kconfig fragment, for one reason -- every
# other image must come out byte for byte as it did before this target existed, which is checked by
# building firmware_tof_cliff on either side of the change and comparing zephyr.bin.
#
# COMPARE zephyr.bin AND NOT zephyr.signed.bin. The signature uses PSS, whose salt is random, so two
# signings of identical bytes differ; the raw image is the reproducible artefact.
#
# THIS TARGET IS THE BENCH IMAGE, and it says so on the command line rather than in a default:
# TOF_AUTO_COMMISSION_PROFILE lets the board entertain a commissioning request at all, and
# TOF_AUTO_COMMISSION_PERMIT_ENUMERATION lets it re-enumerate the chain when it gets one. Both are
# off in every other build, and there is no runtime way to turn either on -- what a board will do is
# fixed by the image it is running and is visible in this line.
#
# A trustworthy stationary condition does not exist yet; it is an open item against safety. Permitting
# enumeration here is a bench decision, and an image built this way must not go on a machine that can
# move until that item is closed.
#
# MEASURED, 2026-09-19, at the commit that added the call site. zephyr.bin is what is compared --
# zephyr.signed.bin uses PSS, whose salt is random, so two signings of identical bytes differ.
#
#   image             zephyr.bin   signed.bin      RAM     FLASH
#   product              194,748      195,084  181,440    74.29%   byte-identical to the baseline
#   cliff                248,716      249,052  220,736    94.88%   byte-identical to the baseline
#   auto-commission      255,952      256,288  226,240    97.64%
#
#   auto-commission over cliff:   +7,236 B flash,  +5,504 B RAM
#   headroom against the 261,712 B signed ceiling:   5,424 B
#
# 4,160 B of the RAM delta is the worker stack (worker_stack, 0x1040 in .bss including Zephyr's
# guard). The binding is no longer collected: 49 text symbols from the four commissioning
# translation units are in the ELF, where before the call site existed the linker dropped them.
#
# 97.64% OF THE SLOT. The next thing added to this image has 5,424 B to fit in, and the worker stack
# is a starting point rather than a measurement -- CONFIG_THREAD_ANALYZER on a board will say whether
# 4 KiB is right, and either direction moves this number.
.PHONY: firmware_auto_commission
firmware_auto_commission:
	./scripts/manage_zephyr_patches.sh verify
	$(RUNNER) west zephyr-export
	$(RUNNER) west build -p auto -b lexxpluss_scb lexxpluss_apps -d build-auto-commission -- -DENABLE_TOF_CHAIN=1 -DENABLE_TOF_CLIFF_ULD=ON -DENABLE_TOF_AUTO_COMMISSION=1 -DTOF_AUTO_COMMISSION_PROFILE=1 -DTOF_AUTO_COMMISSION_PERMIT_ENUMERATION=1 -DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1 "-DEXTRA_DTC_OVERLAY_FILE=overlays/tof_chain.overlay;overlays/auto_commission.overlay" -DEXTRA_CONF_FILE=overlays/auto_commission.conf -DCONFIG_STREAM_FLASH=y -DCONFIG_IMG_MANAGER=y -DBOARD_ROOT=/${WORKDIR}/extra -DZEPHYR_EXTRA_MODULES=/${WORKDIR}/extra -DVERSION=${VERSION}
	mv build-auto-commission/zephyr/zephyr.signed.bin out/zephyr_auto_commission.signed.bin
	mv build-auto-commission/zephyr/zephyr.signed.confirmed.bin out/zephyr_auto_commission.signed.confirmed.bin
	cp out/zephyr_auto_commission.signed.confirmed.bin out/zephyr_auto_commission.test.bin
	printf '\377' | dd of=out/zephyr_auto_commission.test.bin bs=1 seek=$$(($$(stat -c%s out/zephyr_auto_commission.test.bin) - 24)) conv=notrunc status=none

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
