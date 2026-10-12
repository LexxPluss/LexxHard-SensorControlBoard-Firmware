#!/usr/bin/env python3
# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
"""Gate the call sites that no unit test can see.

Three of the properties this round is built on are claims about WHERE something is called, not about
what a function computes, so a host test cannot reach them:

  * the quiesce record is written by the quiesce hook and by nothing else -- a second writer is how a
    later probe or stop comes to overwrite the observation the record exists to keep;
  * nothing in the application clears it, because a clear outside a test is indistinguishable from a
    quiesce that was never recorded;
  * the quiesce is timed from a 64-bit monotonic source, not the 32-bit cycle counter that wrapped
    every 19.884 s.

The first version of this gate was weaker than the sentence it was written to defend: it counted one
call anywhere in the adapter, so moving the write into the probe hook passed it, and a clear added
to the probe was not looked for at all. Review found that. This one locates the hook body and checks
inside it.

Run standalone with the source root as the argument; it is also invoked from CMakeLists.txt, so a
tree that violates any of this fails to configure rather than failing in the field.
"""

import re
import sys
from pathlib import Path

ADAPTER = "tof_package_bench.cpp"
WRITER = "tof_l7_quiesce::remember("
CLEARER = "tof_l7_quiesce::forget_all("
BANNED_CLOCK = "k_cycle_get_32"


def code_only(text):
    """The source with comments removed.

    The first run of this gate failed on its OWN explanatory comment, which names the banned clock
    in order to say why it is banned. A check that reads a file must not be satisfiable or defeated
    by prose in it -- the driver half of this project learned the same thing about a structural
    check that matched the comment it was documented by. String and character literals are tracked
    so that a `//` inside a message is not mistaken for a comment.
    """
    out = []
    i = 0
    n = len(text)
    in_string = in_char = in_line = in_block = False
    while i < n:
        c = text[i]
        pair = text[i : i + 2]
        if in_line:
            if c == "\n":
                in_line = False
                out.append(c)
            i += 1
        elif in_block:
            if pair == "*/":
                in_block = False
                i += 2
            else:
                if c == "\n":
                    out.append(c)
                i += 1
        elif in_string or in_char:
            out.append(c)
            if c == "\\":
                if i + 1 < n:
                    out.append(text[i + 1])
                i += 2
                continue
            if (in_string and c == '"') or (in_char and c == "'"):
                in_string = in_char = False
            i += 1
        elif pair == "//":
            in_line = True
            i += 2
        elif pair == "/*":
            in_block = True
            i += 2
        else:
            if c == '"':
                in_string = True
            elif c == "'":
                in_char = True
            out.append(c)
            i += 1
    return "".join(out)


def body_of(text, signature):
    """The text between a function's opening line and the closing brace in column one."""
    start = text.find(signature)
    if start < 0:
        return None
    opened = text.find("{", start)
    if opened < 0:
        return None
    end = text.find("\n}", opened)
    if end < 0:
        return None
    return text[opened : end + 2]


def main(src_root):
    src = Path(src_root)
    problems = []

    sources = sorted(src.glob("*.cpp"))
    if not sources:
        return ["no sources found under %s" % src]

    adapter_path = src / ADAPTER
    if not adapter_path.is_file():
        return ["%s is missing, so the call sites it is supposed to hold cannot be checked" % ADAPTER]
    adapter = code_only(adapter_path.read_text())

    quiesce_body = body_of(adapter, "int hook_quiesce(")
    if quiesce_body is None:
        problems.append("hook_quiesce() could not be located in %s, so where the record is written "
                        "cannot be established" % ADAPTER)
        quiesce_body = ""

    inside = quiesce_body.count(WRITER)
    if inside != 1:
        problems.append("expected exactly one %s inside hook_quiesce(), found %d. Zero means "
                        "nothing records the snapshot on the path that takes it; more than one "
                        "means it is written twice for one call." % (WRITER, inside))

    for path in sources:
        text = code_only(path.read_text())
        outside = text.count(WRITER) - (inside if path == adapter_path else 0)
        if outside != 0:
            problems.append("%s calls %s outside hook_quiesce() %d time(s). The record is written "
                            "by the quiesce and by nothing else -- a probe or a stop that writes it "
                            "is exactly how the observation gets overwritten."
                            % (path.name, WRITER, outside))
        if CLEARER in text:
            problems.append("%s calls %s. Only a test may clear the record: a cleared record is "
                            "indistinguishable from a quiesce that never happened." % (path.name, CLEARER))

    if BANNED_CLOCK in adapter:
        problems.append("%s uses %s. It wraps every 19.884 s at 216 MHz and a failing quiesce runs "
                        "longer than that; time the call from a 64-bit monotonic source."
                        % (ADAPTER, BANNED_CLOCK))

    return problems


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: check_call_sites.py <app src dir>", file=sys.stderr)
        sys.exit(2)
    found = main(sys.argv[1])
    for line in found:
        print("call-site gate: %s" % line, file=sys.stderr)
    sys.exit(1 if found else 0)
