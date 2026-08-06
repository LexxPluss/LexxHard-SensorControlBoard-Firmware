#!/usr/bin/env bash
# Copyright (c) 2026, LexxPluss Inc.
# SPDX-License-Identifier: BSD-3-Clause
#
# Applies, verifies or reverses the repository's Zephyr patches.
#
# The west manifest pins Zephyr to an exact tag, and the patches under
# patches/zephyr/ are diffs against exactly that baseline. This script is
# idempotent so the Makefile can call it unconditionally:
#
#   apply    fail unless the checkout is the pinned baseline; then apply
#            every patch that is not already applied (already-applied
#            patches are detected by a reverse-check and skipped)
#   verify   fail unless every patch is currently applied -- production
#            build targets run this first, so building against an
#            unpatched driver fails loudly instead of silently shipping
#            the un-classified error codes
#   unapply  reverse exactly our patches (for `make update`, which must
#            hand west a pristine tree); patches that are not applied are
#            skipped
#
# Anything that matches neither direction (a half-applied or conflicting
# state) is an error in every mode: fix the tree by hand before retrying.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The west workspace root holds zephyr/ next to the manifest repository;
# in a git worktree of the app repo, walk up until zephyr/ is found.
find_zephyr() {
    local d="$ROOT"
    while [ "$d" != "/" ]; do
        if [ -d "$d/zephyr/.git" ] || [ -f "$d/zephyr/.git" ]; then
            echo "$d/zephyr"
            return 0
        fi
        d="$(dirname "$d")"
    done
    echo "zephyr checkout not found above $ROOT" >&2
    return 1
}
ZEPHYR="$(find_zephyr)"
PATCH_DIR="$ROOT/patches/zephyr"
BASELINE="468eb56cf242eedba62006ee758700ee6148763f"  # v3.6.0, as pinned in west.yml

mode="${1:-}"
case "$mode" in
    apply|verify|unapply) ;;
    *) echo "usage: $0 apply|verify|unapply" >&2; exit 2 ;;
esac

head="$(git -C "$ZEPHYR" rev-parse HEAD)"
if [ "$mode" = "apply" ] && [ "$head" != "$BASELINE" ]; then
    echo "zephyr HEAD is $head, expected the pinned baseline $BASELINE" >&2
    echo "(west update to the manifest revision before applying patches)" >&2
    exit 1
fi

status=0
for p in "$PATCH_DIR"/*.patch; do
    [ -e "$p" ] || { echo "no patches in $PATCH_DIR" >&2; exit 1; }
    name="$(basename "$p")"
    if git -C "$ZEPHYR" apply --reverse --check "$p" 2>/dev/null; then
        applied=1
    elif git -C "$ZEPHYR" apply --check "$p" 2>/dev/null; then
        applied=0
    else
        echo "$name: neither applied nor applicable -- the tree is in a mixed state" >&2
        exit 1
    fi
    case "$mode" in
        apply)
            if [ "$applied" = 1 ]; then
                echo "$name: already applied"
            else
                git -C "$ZEPHYR" apply "$p"
                echo "$name: applied"
            fi
            ;;
        verify)
            if [ "$applied" = 1 ]; then
                echo "$name: verified"
            else
                echo "$name: NOT applied -- run 'make setup' (or scripts/manage_zephyr_patches.sh apply)" >&2
                status=1
            fi
            ;;
        unapply)
            if [ "$applied" = 1 ]; then
                git -C "$ZEPHYR" apply --reverse "$p"
                echo "$name: reversed"
            else
                echo "$name: not applied, nothing to reverse"
            fi
            ;;
    esac
done
exit "$status"
