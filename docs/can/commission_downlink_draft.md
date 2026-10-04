# Commissioning downlink — DRAFT, nothing here is frozen

Contract status: **DRAFT.** No golden-vector artefact is generated from this file, neither
repository pins anything in it, and no handler may be wired to a real CAN filter against it. It is
here to make one decision reviewable; the frame layouts below are candidates, not a specification.

The settled part is only the allocation, and it lives in `tof_cliff_wire_contract.md`: the team
allocated `0x218` (request, host to SCB) and `0x219` (status, SCB to host) on 2026-09-19 with
authority over the register. That contract also records that **neither identifier has a payload
layout**, which is what this document would eventually supply.

## The decision this document exists to make

**A transaction status must be attributable to the request that caused it, and today it cannot be.**

The status frame carries `request_seq` and `wire_epoch` and no session identity. The refusal path
makes the gap concrete rather than theoretical — the implementation on the experiment branch answers
a request from an old session by echoing that request's own fields:

    if (req.session_token != token_) {
        out.status = refusal(req.seq, req.wire_epoch, wire::result::stale_session);

So a refusal aimed at a request that is over is indistinguishable, on the wire, from a refusal of
whatever the host is waiting for now.

### Why the existing token does not already cover it

The draft's threat model is explicit that the token is one-directional:

> What the token defends against is **a request from a previous boot arriving after a new one** …
> without a session marker **the SCB** cannot tell that request from a current one.

That protects the SCB from stale requests. There is no counterpart protecting the host from stale
statuses, and the status frame has no field that could provide one.

### The shape of the collision, stated no wider than it is

A host that matches on `request_seq` alone is clearly exposed. A host that also checks `wire_epoch`
is not exposed to the simplest case, because a new session's request normally spends a different
ordinal. **That is not a fix, for two reasons.** The pair recycles — `request_seq` is 8 bits and
`wire_epoch` is the ordinal's low 8 bits — and more importantly **the protocol places no lifetime
bound on a status frame**, so there is no interval after which an old one can be ruled out. A
defence that depends on two 8-bit values not repeating within an unbounded window is a defence with
no stated limit.

The host's own recovery rule makes the first case likelier rather than rarer: when the session token
changes, the sequence allocator "resets to the new session and starts again", so the first request
of a new session routinely carries a sequence number the previous session also used.

## The rule to settle first, before any field budget

**A transaction status must carry the session identity of the request it answers — not the SCB's
current session.**

The distinction is the whole point and is easy to lose. Stamping replies with the current token
would leave the refusal path exactly as broken: a stale-session refusal would carry the new token,
match the host's new pending request, and be accepted as its answer. The current token is already
published separately by the session announcement, which is where a host learns it; a status frame
repeating it adds nothing a host did not have.

Only once that rule is agreed does the field budget below mean anything.

## Candidate layouts, with their costs

The status frame is eight bytes and currently has no spare:

    0 version   1 kind   2 request_seq   3 wire_epoch
    4 phase     5 result 6 stage         7 detail

Measured cardinalities: `phase` 6 values, `result` 18, `wire_stage` 12, `wire_detail` 11 — so 3, 5,
4 and 4 bits respectively.

### A — pack the enums, carry a 16-bit session tag

    0 version   1 kind   2 request_seq        3 wire_epoch
    4 phase:3 | result:5                      5 stage:4 | detail:4
    6–7 session_tag, u16, low 16 bits of the token the REQUEST carried

Keeps every field. Collision between two sessions is 1 in 65,536 rather than the current certainty.
**Cost:** the layout becomes coupled to enum cardinality — a nineteenth `result` still fits, a
thirty-third does not, and the failure would be at generation time rather than on the wire only if a
check is written for it. That check would have to be part of the generator, like the cliff
contract's verdict parity.

### B — drop `wire_epoch` from the status, carry an 8-bit tag

    0 version   1 kind   2 request_seq   3 session_tag (u8)
    4 phase     5 result 6 stage         7 detail

No packing, so no cardinality coupling. **Cost:** 1 in 256 per session pair, and the host loses the
epoch cross-check it would otherwise have had — two weakenings to buy simplicity.

### C — two status frames, no packing

A full `u32` token with every field unpacked, split across two frames. **Cost:** the status stops
being a single atomic frame, which introduces ordering and partial-delivery rules that the
single-frame design does not have, on a path whose whole job is to be unambiguous.

**No recommendation is made here.** A is the only candidate that keeps every field and a
non-trivial tag, and its cost is a generator check that this repository already knows how to write;
that is an argument, not a decision.

## What this document corrects about the working draft

Four statements in the working notes describe a state that has moved. Recorded so the eventual
specification is not written from them.

- **The identifiers are not pending registration.** `0x218`/`0x219` were allocated by the team on
  2026-09-19. This closes nothing about the other CAN identifiers, whose register rows are a
  separate open item.
- **The entropy source is implemented on the experiment branch, not delivered in production.** The
  build asserts the STM32 hardware RNG is enabled and rejects three PRNG fallbacks by name, and an
  overlay exists. None of it is in `prod`, and its cost against the 261,712-byte ceiling is still
  unmeasured.
- **Which side issues the epoch is a deployment decision.** The wire contract requires a persistent
  issuer that cannot reuse a value and says either side may hold it; an earlier revision demanded a
  firmware-side one and withdrew it. The host-issued route is the one release and safety approved
  for the commissioning profile — current, not permanent.
- **There is no host implementation to compare against.** Checked on the LexxAuto branches
  available here: nothing implements this downlink. So the eventual specification can be verified
  against the firmware and a shared vector set, and cannot claim two independent implementations
  agree.

## Out of scope here, and staying that way until the rule above is settled

No CAN filter is registered, no worker starts, nothing touches a sensor, no build option is added,
and the PROVEN clamp is untouched. The measurement and health frames are not modified: this document
adds no byte to `0x216` or `0x217`.
