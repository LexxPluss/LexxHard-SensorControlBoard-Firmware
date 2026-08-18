#!/bin/bash
# Enforces the repository's language boundary.
#
# The rule, and the distinction it rests on:
#
#   Python MAY live in docs/can/ as offline build tooling -- the golden-vector
#   generators and the contract drift check. Those have to be in the repository, or
#   the artefacts cannot be reproduced or verified from a clean checkout.
#
#   Python MUST NOT reach the product: not the application sources, not the product
#   tests, and not any list of sources that gets compiled into an image. Product code
#   and executable tests are C and C++.
#
# So a .py file entering the production Git branch is expected and fine. A .py file
# participating in a Zephyr build, or running on a robot, is not. This script checks
# the second thing, which is the one that matters, because the first is not a defect
# and refusing it would cost the drift check that catches an edited contract with
# stale vectors.
#
# Tracked files only: build directories are full of generated Python from the SDK and
# have nothing to do with what we ship.

set -eu

TOOLING_DIR="docs/can"
failed=0

note() { printf '%s\n' "$*" >&2; }

fail() {
    note "language boundary violation: $1"
    failed=1
}

# ---------------------------------------------------------------- rule 1 -----
# Every tracked Python file lives under the tooling directory.
while IFS= read -r f; do
    [ -n "$f" ] || continue
    case "$f" in
        "$TOOLING_DIR"/*) ;;
        *) fail "$f is Python outside $TOOLING_DIR/" ;;
    esac
done <<< "$(git ls-files '*.py' || true)"

# ---------------------------------------------------------------- rule 2 -----
# No build file may reference a Python file. This is what would pull one into a
# compile, a link or an image.
build_files=$(git ls-files 'CMakeLists.txt' '*/CMakeLists.txt' '*.cmake' || true)
if [ -n "$build_files" ]; then
    while IFS= read -r f; do
        [ -n "$f" ] || continue
        if grep -Iq '\.py\b' "$f" 2>/dev/null; then
            fail "$f references a .py file from a build description"
            grep -In '\.py\b' "$f" | sed 's/^/    /' >&2 || true
        fi
    done <<< "$build_files"
fi

# ---------------------------------------------------------------- rule 3 -----
# Nothing in the application tree -- sources or product tests -- may reference one
# either. A test that shells out to a generator is still a test that needs Python at
# run time.
app_files=$(git ls-files 'lexxpluss_apps' 'extra' 'third_party' || true)
if [ -n "$app_files" ]; then
    while IFS= read -r f; do
        [ -n "$f" ] || continue
        [ -f "$f" ] || continue
        if grep -Iq '\.py\b' "$f" 2>/dev/null; then
            fail "$f references a .py file from the application tree"
            grep -In '\.py\b' "$f" | sed 's/^/    /' >&2 || true
        fi
    done <<< "$app_files"
fi

# ---------------------------------------------------------------- rule 4 -----
# Production sources may include the contract's production header, but not its test
# artefacts.
#
# lexxpluss_apps/CMakeLists.txt puts the whole of docs/can on the application include
# path, because that is how production code reaches tof_cliff_contract.h without a
# vendored copy that could drift. The side effect is that the vector header is reachable
# too -- one #include away from a product build carrying 87 test vectors and a
# RELEASE_FORBIDDEN banner. "Production only uses contract.h" was a comment; this makes
# it a rule.
TEST_ONLY_ARTEFACTS='tof_cliff_contract_vectors\.h|tof_contract_vectors\.h'
src_files=$(git ls-files 'lexxpluss_apps/src' || true)
if [ -n "$src_files" ]; then
    while IFS= read -r f; do
        [ -n "$f" ] || continue
        [ -f "$f" ] || continue
        if grep -IqE "^[[:space:]]*#[[:space:]]*include.*($TEST_ONLY_ARTEFACTS)" "$f" 2>/dev/null; then
            fail "$f includes a test-vector header from production source"
            grep -InE "^[[:space:]]*#[[:space:]]*include.*($TEST_ONLY_ARTEFACTS)" "$f" |
                sed 's/^/    /' >&2 || true
        fi
    done <<< "$src_files"
fi

if [ "$failed" -ne 0 ]; then
    note ""
    note "Python is offline tooling under $TOOLING_DIR/ only. It must not participate in"
    note "a Zephyr build or run on a robot. Move the logic to C/C++, or invoke the tool"
    note "from a Makefile target that is not part of any firmware build."
    note ""
    note "Production sources take the contract's production header (tof_cliff_contract.h)"
    note "and never its test vectors, even though docs/can is on the include path."
    exit 1
fi

py_count=$(git ls-files "$TOOLING_DIR/*.py" | wc -l | tr -d ' ')
printf 'language boundary OK: %s Python tool(s) in %s/, none reachable from a build\n' \
    "$py_count" "$TOOLING_DIR"
