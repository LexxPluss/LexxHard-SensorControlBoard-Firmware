'use strict';

// Numbers without leading zeros.
const NUM = '(0|[1-9]\\d*)';
const V_RE = new RegExp(`^v${NUM}\\.${NUM}\\.${NUM}$`);
const V_SUFFIX_RE = new RegExp(`^v${NUM}\\.${NUM}\\.${NUM}-(.+)$`);
const PROD_RE = new RegExp(`^prod/([A-Za-z0-9][A-Za-z0-9-]*)/v${NUM}\\.${NUM}\\.${NUM}$`);

function invalid(reason) {
  return { cls: 'invalid', reason };
}

function versioned(cls, m, offset) {
  // Digit strings are kept as written; they are never converted to numbers.
  const [major, minor, patch] = [m[offset], m[offset + 1], m[offset + 2]];
  return { cls, major, minor, patch, version: `${major}.${minor}.${patch}` };
}

function parseTag(tag) {
  if (typeof tag !== 'string') {
    return invalid('tag is not a string');
  }
  let m = V_RE.exec(tag);
  if (m) {
    return versioned('v', m, 1);
  }
  m = V_SUFFIX_RE.exec(tag);
  if (m) {
    return { ...versioned('v-suffix', m, 1), suffix: m[4] };
  }
  m = PROD_RE.exec(tag);
  if (m) {
    return { ...versioned('prod', m, 2), id: m[1] };
  }
  return invalid(`tag '${tag}' is not vX.Y.Z, vX.Y.Z-suffix or prod/<id>/vX.Y.Z`);
}

function asList(list) {
  return Array.isArray(list) ? list : [];
}

// First non-zero component difference decides.
function compare(a, b) {
  for (const key of ['major', 'minor', 'patch']) {
    // No leading zeros, so a longer digit string is larger; equal lengths compare lexicographically.
    const x = a[key];
    const y = b[key];
    if (x !== y) {
      return x.length !== y.length ? x.length - y.length : (x < y ? -1 : 1);
    }
  }
  return 0;
}

// Highest numeric version among published, non-pre-release `vMAJOR.MINOR.PATCH` Releases.
function latestTarget(list) {
  let best = null;
  let bestParsed = null;
  for (const r of asList(list)) {
    if (!r || r.draft || r.prerelease) {
      continue;
    }
    const p = parseTag(r.tag_name);
    if (p.cls !== 'v') {
      continue;
    }
    if (bestParsed === null || compare(p, bestParsed) > 0) {
      best = r.tag_name;
      bestParsed = p;
    }
  }
  return best;
}

function result(action, releaseId, makeLatest, reason) {
  return { action, releaseId, makeLatest, reason };
}

function decideAction(tag, listBefore) {
  const parsed = parseTag(tag);
  if (parsed.cls === 'invalid') {
    return result('fail', null, null, parsed.reason);
  }
  if (!Array.isArray(listBefore)) {
    return result('fail', null, null, 'Release list is not an array');
  }
  const list = listBefore.filter((r) => r);
  const same = list.filter((r) => r.tag_name === tag);
  const drafts = same.filter((r) => r.draft);
  // A published Release (a pre-release included) of the tag wins over any draft.
  if (same.length > drafts.length) {
    return result('skip', null, null, null);
  }
  if (drafts.length > 1) {
    return result('fail', null, null, `${drafts.length} draft Releases exist for tag '${tag}'`);
  }
  const draft = drafts[0];
  if (draft && typeof draft.id !== 'number') {
    return result('fail', null, null, `draft Release of tag '${tag}' has no numeric id`);
  }
  // The Release this run makes is published; it is a pre-release by the draft flag or a suffix tag.
  const made = {
    tag_name: tag,
    draft: false,
    prerelease: Boolean(draft && draft.prerelease) || parsed.cls === 'v-suffix',
  };
  const others = list.filter((r) => r !== draft);
  const makeLatest = latestTarget([...others, made]) === tag;
  return draft
    ? result('publish', draft.id, makeLatest, null)
    : result('create', null, makeLatest, null);
}

module.exports = { parseTag, decideAction, latestTarget };
