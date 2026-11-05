'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const impl = require('./release-handling.js');

function deepFreeze(value) {
  if (value !== null && typeof value === 'object' && !Object.isFrozen(value)) {
    Object.freeze(value);
    Object.values(value).forEach(deepFreeze);
  }
  return value;
}

// Every list handed to the module is deep-frozen, so a function that mutates its input throws a TypeError.
function freezeList(list) {
  return deepFreeze(list);
}

const parseTag = (tag) => impl.parseTag(tag);
const decideAction = (tag, list) => impl.decideAction(tag, freezeList(list));
const latestTarget = (list) => impl.latestTarget(freezeList(list));

// Provisional field name of the identifier after `prod/`; the only place to rename it.
const ID_FIELD = 'id';

const ACTIONS = ['skip', 'publish', 'create', 'fail'];
const RESULT_KEYS = ['action', 'makeLatest', 'reason', 'releaseId'];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

let nextId = 1000;
function rel(tag, { draft = false, prerelease = false, id } = {}) {
  const releaseId = id === undefined ? nextId++ : id;
  return { id: releaseId, tag_name: tag, draft, prerelease };
}

function isSuffixTag(tag) {
  return oracleParse(tag).cls === 'v-suffix';
}

// The Release this run makes: published, pre-release when the flag is set or the tag has a suffix.
function madeRelease(tag, draftRel) {
  return { id: 999999, tag_name: tag, draft: false, prerelease: (draftRel ? draftRel.prerelease : false) || isSuffixTag(tag) };
}

// listBefore plus this run's Release (a published draft is replaced by the made Release).
function withMade(list, tag, draftRel) {
  const rest = draftRel ? list.filter((r) => r.id !== draftRel.id) : list;
  return [...rest, madeRelease(tag, draftRel)];
}

function assertFail(result) {
  assert.equal(result.action, 'fail');
  assert.equal(result.releaseId, null);
  assert.equal(typeof result.reason, 'string');
  assert.ok(result.reason.length > 0);
}

function assertInvalid(tag) {
  assert.equal(parseTag(tag).cls, 'invalid');
  assertFail(decideAction(tag, []));
}

// ---------------------------------------------------------------------------
// Independent ORACLE: naive reference of rules Q1-Q16, written without the module under test.
// ---------------------------------------------------------------------------

const NUM = '(0|[1-9]\\d*)';
const ORACLE_V = new RegExp(`^v${NUM}\\.${NUM}\\.${NUM}$`);
const ORACLE_V_SUFFIX = new RegExp(`^v${NUM}\\.${NUM}\\.${NUM}-[A-Za-z0-9.-]+$`);
const ORACLE_PROD = new RegExp(`^prod/[A-Za-z0-9][A-Za-z0-9-]*/v${NUM}\\.${NUM}\\.${NUM}$`);

function oracleParse(tag) {
  const m = ORACLE_V.exec(tag);
  if (m) {
    return { cls: 'v', ver: [BigInt(m[1]), BigInt(m[2]), BigInt(m[3])] };
  }
  if (ORACLE_V_SUFFIX.test(tag)) {
    return { cls: 'v-suffix' };
  }
  if (ORACLE_PROD.test(tag)) {
    return { cls: 'prod' };
  }
  return { cls: 'invalid' };
}

function oracleLatest(list) {
  let best = null;
  let bestVer = null;
  for (const r of list) {
    if (r.draft || r.prerelease) continue;
    const p = oracleParse(r.tag_name);
    if (p.cls !== 'v') continue;
    const [a, b, c] = p.ver;
    if (
      bestVer === null ||
      a > bestVer[0] ||
      (a === bestVer[0] && (b > bestVer[1] || (b === bestVer[1] && c > bestVer[2])))
    ) {
      best = r.tag_name;
      bestVer = p.ver;
    }
  }
  return best;
}

function oracleDecide(tag, list) {
  if (oracleParse(tag).cls === 'invalid') {
    return { action: 'fail', releaseId: null };
  }
  const same = list.filter((r) => r.tag_name === tag);
  const published = same.filter((r) => !r.draft);
  const drafts = same.filter((r) => r.draft);
  if (published.length > 0) {
    return { action: 'skip', releaseId: null };
  }
  if (drafts.length >= 2) {
    return { action: 'fail', releaseId: null };
  }
  const draftRel = drafts.length === 1 ? drafts[0] : undefined;
  const makeLatest = oracleLatest(withMade(list, tag, draftRel)) === tag;
  return draftRel
    ? { action: 'publish', releaseId: draftRel.id, makeLatest }
    : { action: 'create', releaseId: null, makeLatest };
}

// ---------------------------------------------------------------------------
// Invariant checks shared by I1-I5 and the random streams
// ---------------------------------------------------------------------------

function checkI1(tag, list, res, where) {
  assert.ok(ACTIONS.includes(res.action), `I1 ${where}: unknown action ${res.action}`);
  assert.deepEqual(Object.keys(res).sort(), RESULT_KEYS, `I1 ${where}: result shape`);
}

function checkI2(tag, list, res, where) {
  const before = list.filter((r) => r.tag_name === tag).length;
  if (res.action === 'create') {
    assert.equal(before, 0,`I2 ${where}: create over an existing Release`);
  } else if (res.action === 'publish') {
    const draftRel = list.find((r) => r.id === res.releaseId);
    assert.ok(draftRel && draftRel.draft && draftRel.tag_name === tag, `I2 ${where}: publish target`);
    assert.equal(before, 1, `I2 ${where}: publish with other Releases of the tag`);
  } else if (res.action === 'skip') {
    assert.equal(res.releaseId, null, `I2 ${where}: skip names a release`);
  } else {
    assert.equal(res.releaseId, null, `I2 ${where}: fail names a release`);
  }
}

function checkI3I5(tag, list, res, where) {
  if (res.action !== 'create' && res.action !== 'publish') return;
  const draftRel = res.action === 'publish' ? list.find((r) => r.id === res.releaseId) : undefined;
  const expected = latestTarget(withMade(list, tag, draftRel)) === tag;
  assert.equal(typeof res.makeLatest, 'boolean', `I3 ${where}: makeLatest type`);
  assert.equal(res.makeLatest, expected, `I3/I5 ${where}: makeLatest vs Latest target`);
  if (tag.startsWith('prod/') || isSuffixTag(tag) || (draftRel && draftRel.prerelease)) {
    assert.equal(res.makeLatest, false, `I3 ${where}: prod or pre-release made Latest`);
  }
}

function checkI4(tag, list, res, where) {
  const target = latestTarget(list);
  assert.ok(target === null || !target.startsWith('prod/'), `I4 ${where}: prod target ${target}`);
  if (tag.startsWith('prod/') && (res.action === 'create' || res.action === 'publish')) {
    assert.equal(res.makeLatest, false, `I4 ${where}: prod made Latest`);
    assert.notEqual(latestTarget(withMade(list, tag, undefined)), tag, `I4 ${where}: prod is the target`);
  }
}

// ---------------------------------------------------------------------------
// Group D: action for the tag
// ---------------------------------------------------------------------------

test('D1: no Release for a v tag -> create; makeLatest iff the made Release is the highest version', () => {
  const highest = decideAction('v3.8.0', [rel('v3.7.2')]);
  assert.equal(highest.action, 'create');
  assert.equal(highest.makeLatest, true);
  const lower = decideAction('v3.8.0', [rel('v3.9.0')]);
  assert.equal(lower.action, 'create');
  assert.equal(lower.makeLatest, false);
});

test('D2: no Release for a prod tag -> create with makeLatest false', () => {
  const res = decideAction('prod/acme/v1.0.0', [rel('v3.8.0')]);
  assert.equal(res.action, 'create');
  assert.equal(res.makeLatest, false);
});

test('D3: published Release only, v class -> skip, nothing to publish', () => {
  const res = decideAction('v3.8.0', [rel('v3.8.0')]);
  assert.equal(res.action, 'skip');
  assert.equal(res.releaseId, null);
});

test('D4: published Release only, prod class -> skip, nothing to publish', () => {
  const res = decideAction('prod/acme/v1.0.0', [rel('prod/acme/v1.0.0'), rel('v3.8.0')]);
  assert.equal(res.action, 'skip');
  assert.equal(res.releaseId, null);
});

test('D5: draft of the same tag only, v class -> publish that draft; makeLatest iff highest', () => {
  const draft = rel('v3.9.0', { draft: true, id: 77 });
  const top = decideAction('v3.9.0', [draft, rel('v3.8.0')]);
  assert.equal(top.action, 'publish');
  assert.equal(top.releaseId, 77);
  assert.equal(top.makeLatest, true);
  const lowerDraft = rel('v3.7.2', { draft: true, id: 78 });
  const low = decideAction('v3.7.2', [lowerDraft, rel('v3.8.0')]);
  assert.equal(low.action, 'publish');
  assert.equal(low.releaseId, 78);
  assert.equal(low.makeLatest, false);
});

test('D6: draft and published Release of the same tag -> skip, draft not published', () => {
  const res = decideAction('v3.8.0', [rel('v3.8.0', { draft: true }), rel('v3.8.0')]);
  assert.equal(res.action, 'skip');
  assert.equal(res.releaseId, null);
});

test('D7: stale draft of another tag, no Release for the tag -> create, stale draft untouched', () => {
  const res = decideAction('v3.8.0', [rel('v3.9.0', { draft: true, id: 5 })]);
  assert.equal(res.action, 'create');
  assert.equal(res.releaseId, null);
});

test('D8: stale draft and a draft of the same tag -> the same-tag draft is published', () => {
  const res = decideAction('v3.8.0', [rel('v3.9.0', { draft: true, id: 5 }), rel('v3.8.0', { draft: true, id: 6 })]);
  assert.equal(res.action, 'publish');
  assert.equal(res.releaseId, 6);
});

test('D9: two drafts of the same tag -> fail; Latest target computed without this run', () => {
  const list = [rel('v3.9.0', { draft: true }), rel('v3.9.0', { draft: true }), rel('v3.8.0')];
  const res = decideAction('v3.9.0', list);
  assertFail(res);
  assert.equal(res.makeLatest, null);
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('D10: prod draft of the same tag -> publish with makeLatest false; not the Latest target', () => {
  const list = [rel('prod/acme/v1.0.0', { draft: true, id: 9 }), rel('v3.8.0')];
  const res = decideAction('prod/acme/v1.0.0', list);
  assert.equal(res.action, 'publish');
  assert.equal(res.releaseId, 9);
  assert.equal(res.makeLatest, false);
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('D11: published pre-release of the tag counts as published -> skip', () => {
  const flagged = decideAction('v3.9.0', [rel('v3.9.0', { prerelease: true })]);
  assert.equal(flagged.action, 'skip');
  assert.equal(flagged.releaseId, null);
  const suffixed = decideAction('v3.9.0-rc1', [rel('v3.9.0-rc1', { prerelease: true })]);
  assert.equal(suffixed.action, 'skip');
  assert.equal(suffixed.releaseId, null);
});

test('D12: draft that is a pre-release (flag or suffix tag) -> publish with makeLatest false', () => {
  const flagDraft = rel('v3.9.0', { draft: true, prerelease: true, id: 21 });
  const flagRes = decideAction('v3.9.0', [flagDraft, rel('v3.8.0')]);
  assert.equal(flagRes.action, 'publish');
  assert.equal(flagRes.releaseId, 21);
  assert.equal(flagRes.makeLatest, false);
  assert.notEqual(latestTarget(withMade([flagDraft, rel('v3.8.0')], 'v3.9.0', flagDraft)), 'v3.9.0');

  assert.equal(parseTag('v3.9.0-rc1').cls, 'v-suffix');
  const sufDraft = rel('v3.9.0-rc1', { draft: true, id: 22 });
  const sufRes = decideAction('v3.9.0-rc1', [sufDraft, rel('v3.8.0')]);
  assert.equal(sufRes.action, 'publish');
  assert.equal(sufRes.releaseId, 22);
  assert.equal(sufRes.makeLatest, false);
  assert.equal(latestTarget(withMade([sufDraft, rel('v3.8.0')], 'v3.9.0-rc1', sufDraft)), 'v3.8.0');
});

test('D13: suffix tag, no Release, published v3.8.0 -> create with makeLatest false; target stays v3.8.0', () => {
  const list = [rel('v3.8.0')];
  const res = decideAction('v3.9.0-rc1', list);
  assert.equal(res.action, 'create');
  assert.equal(res.makeLatest, false);
  assert.equal(latestTarget(withMade(list, 'v3.9.0-rc1', undefined)), 'v3.8.0');
});

test('D14: invalid tag, any list -> fail; target computed over the existing list', () => {
  const list = [rel('v3.8.0'), rel('v3.9.0', { draft: true })];
  const res = decideAction('v3.9', list);
  assertFail(res);
  assert.equal(res.makeLatest, null);
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('D15: a list that is not an array -> fail with reason', () => {
  for (const bad of [undefined, null, 'releases', { length: 0 }]) {
    const res = decideAction('v3.9.0', bad);
    assertFail(res);
    assert.equal(res.makeLatest, null);
  }
});

test('D16: same-tag draft without a numeric id -> fail with reason', () => {
  const noId = { tag_name: 'v3.9.0', draft: true, prerelease: false };
  const strId = { id: '12', tag_name: 'v3.9.0', draft: true, prerelease: false };
  for (const draft of [noId, strId]) {
    const res = decideAction('v3.9.0', [draft, rel('v3.8.0')]);
    assertFail(res);
    assert.equal(res.makeLatest, null);
  }
});

// ---------------------------------------------------------------------------
// Group L: Latest correction
// ---------------------------------------------------------------------------

test('L1: prod Release published as Latest in the UI -> target is the correct v Release', () => {
  const list = [rel('v3.8.0'), rel('prod/acme/v1.0.0')];
  assert.equal(decideAction('prod/acme/v1.0.0', list).action, 'skip');
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('L2: creation skipped, Latest already correct -> same target (Latest is never an input)', () => {
  const list = [rel('v3.8.0')];
  assert.equal(decideAction('v3.8.0', list).action, 'skip');
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('L3: no v Release exists -> no target, no failure', () => {
  assert.equal(latestTarget([]), null);
  assert.equal(latestTarget([rel('prod/acme/v1.0.0')]), null);
});

test('L4: only pre-release v Releases -> no target', () => {
  const list = [rel('v3.8.0', { prerelease: true }), rel('v3.9.0', { prerelease: true })];
  assert.equal(latestTarget(list), null);
});

test('L5: highest version wins over the later-published one', () => {
  const list = [
    { ...rel('v3.8.0'), published_at: '2026-01-01T00:00:00Z' },
    { ...rel('v3.7.2'), published_at: '2026-06-01T00:00:00Z' },
  ];
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('L6: a newer draft is never chosen; newest published non-pre-release is the target', () => {
  const list = [rel('v3.9.0', { draft: true }), rel('v3.8.0'), rel('v3.7.0')];
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('L7: pushed v3.9.0 with published v3.8.0 -> created with makeLatest true; target v3.9.0', () => {
  const list = [rel('v3.8.0')];
  const res = decideAction('v3.9.0', list);
  assert.equal(res.action, 'create');
  assert.equal(res.makeLatest, true);
  assert.equal(latestTarget(withMade(list, 'v3.9.0', undefined)), 'v3.9.0');
});

test('L8: pushed v3.7.2 with published v3.8.0 -> created with makeLatest false; target stays v3.8.0', () => {
  const list = [rel('v3.8.0')];
  const res = decideAction('v3.7.2', list);
  assert.equal(res.action, 'create');
  assert.equal(res.makeLatest, false);
  assert.equal(latestTarget(withMade(list, 'v3.7.2', undefined)), 'v3.8.0');
});

test('L9: v3.10.0 beats v3.9.0 (numeric, not string comparison)', () => {
  assert.equal(latestTarget([rel('v3.9.0'), rel('v3.10.0')]), 'v3.10.0');
  assert.equal(latestTarget([rel('v3.10.0'), rel('v3.9.0')]), 'v3.10.0');
});

test('L10: suffix tag with prerelease flag false is not a target', () => {
  const list = [rel('v3.9.0-rc1', { prerelease: false }), rel('v3.8.0')];
  assert.equal(latestTarget(list), 'v3.8.0');
});

test('L11: a list that is not an array -> no target', () => {
  assert.equal(latestTarget(null), null);
  assert.equal(latestTarget(undefined), null);
  assert.equal(latestTarget('releases'), null);
  assert.equal(latestTarget({ length: 1, 0: rel('v3.8.0') }), null);
});

// ---------------------------------------------------------------------------
// Group T: tag classification
// ---------------------------------------------------------------------------

test('T1: v3.8.0 is class v', () => {
  assert.equal(parseTag('v3.8.0').cls, 'v');
});

test('T2: real prod tag, dashes inside the id', () => {
  const p = parseTag('prod/pj-paco-xxxx-joint-1/v0.1.0');
  assert.equal(p.cls, 'prod');
  assert.equal(p[ID_FIELD], 'pj-paco-xxxx-joint-1');
});

test('T3: slash inside the id is invalid', () => {
  assertInvalid('prod/x/y/v1.0.0');
});

test('T4: v tag with suffix is class v-suffix', () => {
  assert.equal(parseTag('v3.4.0-rc1-fat').cls, 'v-suffix');
});

test('T5: real prod tag parse and artifact version', () => {
  const p = parseTag('prod/pj-paco-xxxx-joint-1/v0.1.0');
  assert.equal(p.cls, 'prod');
  assert.equal(p[ID_FIELD], 'pj-paco-xxxx-joint-1');
  assert.equal(p.version, '0.1.0');
  assert.equal(`${p[ID_FIELD]}-v${p.version}`, 'pj-paco-xxxx-joint-1-v0.1.0');
});

test('T6: multi-digit components', () => {
  const p = parseTag('prod/acme/v12.34.56');
  assert.equal(p.cls, 'prod');
  assert.equal(p[ID_FIELD], 'acme');
  assert.equal(p.version, '12.34.56');
  assert.equal(`${p[ID_FIELD]}-v${p.version}`, 'acme-v12.34.56');
});

test('T7: prod tag without v is invalid', () => {
  assertInvalid('prod/acme/0.1.0');
});

test('T8: prod tag with a pre-release suffix is invalid', () => {
  assertInvalid('prod/acme/v0.1.0-rc1');
});

test('T9: slash in the id (a/b) is invalid', () => {
  assertInvalid('prod/a/b/v1.0.0');
});

test('T10: two-component prod version is invalid', () => {
  assertInvalid('prod/acme/v1.0');
});

test('T11: v1.0.0 is class v, not prod', () => {
  const p = parseTag('v1.0.0');
  assert.equal(p.cls, 'v');
  assert.notEqual(p.cls, 'prod');
});

test('T12: empty id is invalid', () => {
  assertInvalid('prod//v1.0.0');
});

test('T13: two-component v tag is invalid', () => {
  assertInvalid('v3.9');
});

test('T14: non-numeric v tag is invalid', () => {
  assertInvalid('vfoo');
});

test('T15: leading zeros in version numbers are invalid', () => {
  assertInvalid('v01.2.3');
  assertInvalid('prod/acme/v1.02.3');
});

test('T16: id charset is letters, digits and hyphens, starting with a letter or digit', () => {
  for (const tag of ['prod/-x/v1.0.0', 'prod/a.b/v1.0.0', 'prod/a_b/v1.0.0', 'prod/../v1.0.0']) {
    assertInvalid(tag);
  }
  const upper = parseTag('prod/ACME/v1.0.0');
  assert.equal(upper.cls, 'prod');
  assert.equal(upper[ID_FIELD], 'ACME');
  const real = parseTag('prod/pj-paco-xxxx-joint-1/v0.1.0');
  assert.equal(real.cls, 'prod');
  assert.equal(real[ID_FIELD], 'pj-paco-xxxx-joint-1');
});

test('T17: large numbers keep exact text and exact ordering', () => {
  assert.equal(parseTag('v1.0.99999999999999999999').version, '1.0.99999999999999999999');
  const big = rel('v1.0.99999999999999999999');
  const small = rel('v1.0.99999999999999999998');
  assert.equal(latestTarget([big, small]), 'v1.0.99999999999999999999');
  assert.equal(latestTarget([small, big]), 'v1.0.99999999999999999999');
});

test('T18: suffix charset is [A-Za-z0-9.-]+ (hostile characters and empty suffix are invalid)', () => {
  for (const tag of [
    'v1.2.3-a$b',
    'v1.2.3-a;b',
    'v1.2.3-a(b)',
    'v1.2.3-a"b',
    "v1.2.3-a'b",
    'v1.2.3-a b',
    'v1.2.3-',
    'v1.2.3-a/b',
  ]) {
    assertInvalid(tag);
  }
  const rc = parseTag('v3.4.0-rc1-fat');
  assert.equal(rc.cls, 'v-suffix');
  assert.equal(rc.suffix, 'rc1-fat');
  const dotted = parseTag('v1.2.3-a.b-c');
  assert.equal(dotted.cls, 'v-suffix');
  assert.equal(dotted.suffix, 'a.b-c');
});

// ---------------------------------------------------------------------------
// Invariants over the reduced matrix
// ---------------------------------------------------------------------------

function buildMatrix() {
  const rows = [];
  const tags = ['v3.9.0', 'v3.9.0-rc1', 'prod/acme/v1.0.0'];
  const bases = [
    ['no base', () => []],
    ['base v3.8.0', () => [rel('v3.8.0')]],
    ['base v4.0.0 and v3.8.0', () => [rel('v4.0.0'), rel('v3.8.0')]],
  ];
  const states = [
    ['none', () => []],
    ['draft only', (t) => [rel(t, { draft: true })]],
    ['published only', (t) => [rel(t)]],
    ['both', (t) => [rel(t, { draft: true }), rel(t)]],
  ];
  const others = [
    ['O none', () => []],
    ['O stale', () => [rel('v4.0.0', { draft: true })]],
  ];
  for (const tag of tags) {
    for (const [stateName, state] of states) {
      for (const [otherName, other] of others) {
        for (const [baseName, base] of bases) {
          rows.push({
            where: `${tag} / ${stateName} / ${otherName} / ${baseName}`,
            tag,
            list: [...base(), ...other(), ...state(tag)],
          });
        }
      }
    }
  }
  rows.push({
    where: 'D9 two drafts',
    tag: 'v3.9.0',
    list: [rel('v3.9.0', { draft: true }), rel('v3.9.0', { draft: true }), rel('v3.8.0')],
  });
  rows.push({
    where: 'D11 published pre-release',
    tag: 'v3.9.0',
    list: [rel('v3.9.0', { prerelease: true }), rel('v3.8.0')],
  });
  return rows;
}

function forEachRow(check) {
  for (const row of buildMatrix()) {
    check(row.tag, row.list, decideAction(row.tag, row.list), row.where);
  }
}

test('I1: exactly one of skip, publish, create, fail is chosen (matrix)', () => {
  forEachRow(checkI1);
});

test('I2: after the action never two Releases for one tag (matrix)', () => {
  forEachRow(checkI2);
});

test('I3: makeLatest equals "made Release is the Latest target" (matrix)', () => {
  forEachRow(checkI3I5);
});

test('I4: a prod tag never becomes the re-assertion target (matrix)', () => {
  forEachRow(checkI4);
});

test('I5: makeLatest sent on create or publish equals "this Release is the target"; only the permitted keys (matrix)', () => {
  forEachRow((tag, list, res, where) => {
    checkI1(tag, list, res, where);
    checkI3I5(tag, list, res, where);
  });
});

// ---------------------------------------------------------------------------
// Random streams against the ORACLE
// ---------------------------------------------------------------------------

function mulberry32(seed) {
  let a = seed >>> 0;
  return function next() {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const V_TAGS = ['v3.7.2', 'v3.8.0', 'v3.9.0', 'v3.10.0', 'v4.0.0'];
const SUFFIX_TAGS = ['v3.9.0-rc1', 'v3.4.0-rc1-fat'];
const PROD_TAGS = ['prod/acme/v1.0.0', 'prod/pj-paco-xxxx-joint-1/v0.1.0', 'prod/acme/v12.34.56'];
const INVALID_TAGS = [
  'v3.9',
  'vfoo',
  'prod/acme/v1.0',
  'prod/x/y/v1.0.0',
  'v01.2.3',
  'prod/acme/v1.02.3',
  'prod/-x/v1.0.0',
  'prod/a.b/v1.0.0',
  'prod/a_b/v1.0.0',
  'prod/../v1.0.0',
  'v1.2.3-a$b',
  'v1.2.3-a;b',
  'v1.2.3-a(b)',
  'v1.2.3-a"b',
  "v1.2.3-a'b",
  'v1.2.3-a b',
  'v1.2.3-',
  'v1.2.3-a/b',
];

function pick(rng, arr) {
  return arr[Math.floor(rng() * arr.length)];
}

function randomTag(rng, prodShare, allowInvalid) {
  const r = rng();
  if (allowInvalid && r < 0.1) return pick(rng, INVALID_TAGS);
  if (r < 0.1 + prodShare) return pick(rng, PROD_TAGS);
  return rng() < 0.2 ? pick(rng, SUFFIX_TAGS) : pick(rng, V_TAGS);
}

function runStream(seed, prodShare, iterations) {
  const rng = mulberry32(seed);
  let idCounter = 1;
  const actionCounts = { skip: 0, publish: 0, create: 0, fail: 0 };
  const classCounts = { v: 0, 'v-suffix': 0, prod: 0, invalid: 0 };
  for (let i = 0; i < iterations; i++) {
    const n = Math.floor(rng() * 9);
    const list = [];
    for (let k = 0; k < n; k++) {
      list.push({
        id: idCounter++,
        tag_name: randomTag(rng, prodShare, false),
        draft: rng() < 0.35,
        prerelease: rng() < 0.2,
      });
    }
    const tag = list.length > 0 && rng() < 0.5 ? pick(rng, list).tag_name : randomTag(rng, prodShare, true);
    const where = `seed=${seed} iter=${i} tag=${tag} list=${JSON.stringify(list)}`;

    const res = decideAction(tag, list);
    const want = oracleDecide(tag, list);
    actionCounts[want.action] += 1;
    classCounts[oracleParse(tag).cls] += 1;
    assert.equal(res.action, want.action, `action ${where}`);
    assert.equal(res.releaseId, want.releaseId, `releaseId ${where}`);
    if (want.action === 'create' || want.action === 'publish') {
      assert.equal(res.makeLatest, want.makeLatest, `makeLatest ${where}`);
    }
    if (want.action === 'fail') {
      assert.equal(typeof res.reason, 'string', `reason ${where}`);
    }
    assert.equal(latestTarget(list), oracleLatest(list), `latestTarget ${where}`);

    checkI1(tag, list, res, where);
    checkI2(tag, list, res, where);
    checkI3I5(tag, list, res, where);
    checkI4(tag, list, res, where);

    if (want.action === 'create' || want.action === 'publish') {
      const draftRel = want.action === 'publish' ? list.find((r) => r.id === want.releaseId) : undefined;
      const all = withMade(list, tag, draftRel);
      assert.equal(latestTarget(all), oracleLatest(all), `latestTarget with made ${where}`);
    }
  }
  for (const [name, count] of Object.entries(actionCounts)) {
    assert.ok(count >= 5, `seed=${seed}: action ${name} occurred only ${count} times`);
  }
  for (const [name, count] of Object.entries(classCounts)) {
    assert.ok(count >= 5, `seed=${seed}: tag class ${name} occurred only ${count} times`);
  }
}

test('Z1: seeded random Release lists agree with the oracle and keep I1-I5', () => {
  runStream(20261105, 0.2, 300);
});

test('Z2: second seed with a larger share of prod Releases and drafts agrees with the oracle', () => {
  runStream(3911, 0.6, 300);
});
