'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const run = require('./run.js');

// ---------------------------------------------------------------------------
// Fake github, context and core
// ---------------------------------------------------------------------------

const REPO = { owner: 'acme-org', repo: 'scb' };

let nextId = 5000;
function rel(tag, { draft = false, prerelease = false, id } = {}) {
  const releaseId = id === undefined ? nextId++ : id;
  return { id: releaseId, tag_name: tag, draft, prerelease };
}

// getReleaseByTag answers 404 (numeric .status) unless a published Release of the tag is configured.
// A draft counts as none.
function notFound() {
  const error = new Error('Not Found');
  error.status = 404;
  return error;
}

// Knobs of the fake (all optional, defaults reproduce the plain behaviour):
//   list            Releases returned by paginate(listReleases).
//   tagLookup       getReleaseByTag result. undefined: computed from list (published Release of the tag, else 404);
//                   a Release object: returned as is; null: forced 404.
//   tagLookupError  numeric status (e.g. 500): getReleaseByTag throws an Error with that .status.
function makeFake(list, { tagLookup, tagLookupError } = {}) {
  const calls = { listReleases: [], getReleaseByTag: [], createRelease: [], updateRelease: [] };
  const listReleases = () => {};
  const github = {
    paginate: async (fn, params) => {
      assert.equal(fn, listReleases);
      calls.listReleases.push(params);
      return list.map((r) => ({ ...r }));
    },
    rest: {
      repos: {
        listReleases,
        getReleaseByTag: async (params) => {
          calls.getReleaseByTag.push(params);
          if (tagLookupError !== undefined) {
            const error = new Error(`Lookup failed with ${tagLookupError}`);
            error.status = tagLookupError;
            throw error;
          }
          const found =
            tagLookup === undefined ? list.find((r) => r.tag_name === params.tag && !r.draft) : tagLookup;
          if (!found) {
            throw notFound();
          }
          return { data: found };
        },
        createRelease: async (params) => {
          calls.createRelease.push(params);
          return { data: { id: 9001, tag_name: params.tag_name, draft: false, prerelease: params.prerelease } };
        },
        updateRelease: async (params) => {
          calls.updateRelease.push(params);
          const old = list.find((r) => r.id === params.release_id);
          return {
            data: {
              id: params.release_id,
              tag_name: old ? old.tag_name : '',
              draft: false,
              prerelease: old ? old.prerelease : false,
            },
          };
        },
      },
    },
  };
  const outputs = {};
  const failures = [];
  const infos = [];
  const core = {
    setOutput: (name, value) => {
      outputs[name] = value;
    },
    setFailed: (message) => {
      failures.push(message);
    },
    info: (message) => {
      infos.push(message);
    },
  };
  return { github, context: { repo: REPO }, core, calls, outputs, failures, infos };
}

async function runRelease(tag, list, options = {}) {
  const fake = makeFake(list, options);
  await run({ github: fake.github, context: fake.context, core: fake.core, mode: 'release', tag });
  return fake;
}

async function runReassert(
  tag,
  list,
  { releaseAction = 'skip', releaseId = '', releasePrerelease = '' } = {},
  options = {},
) {
  const fake = makeFake(list, options);
  await run({
    github: fake.github,
    context: fake.context,
    core: fake.core,
    mode: 'reassert',
    tag,
    releaseAction,
    releaseId,
    releasePrerelease,
  });
  return fake;
}

// ---------------------------------------------------------------------------
// Group C: characterization of run.js on head
// ---------------------------------------------------------------------------

test('C1: plain highest tag without a Release is created as Latest (R-A2 (a))', async () => {
  // Measured: created with generated notes, Latest v3.0.0. name and make_latest type: code reading, not measured.
  const fake = await runRelease('v3.0.0', []);
  assert.equal(fake.calls.createRelease.length, 1);
  assert.equal(fake.calls.updateRelease.length, 0);
  const payload = fake.calls.createRelease[0];
  assert.equal(payload.tag_name, 'v3.0.0');
  assert.equal(payload.prerelease, false);
  assert.equal(payload.make_latest, 'true');
  assert.equal(payload.generate_release_notes, true);
  assert.equal(payload.name, 'v3.0.0');
});

test('C2: suffix tag is created as a pre-release and never Latest (code reading, not measured)', async () => {
  const fake = await runRelease('v3.9.0-rc1', [rel('v3.8.0')]);
  assert.equal(fake.calls.createRelease.length, 1);
  const payload = fake.calls.createRelease[0];
  assert.equal(payload.prerelease, true);
  assert.equal(payload.make_latest, 'false');
});

test('C3: lower version never takes Latest on create (R-A2 (d); payload value from code reading)', async () => {
  const fake = await runRelease('v3.0.5', [rel('v3.1.0')]);
  assert.equal(fake.calls.createRelease.length, 1);
  assert.equal(fake.calls.createRelease[0].make_latest, 'false');
});

test('C4: prod tag is created without Latest and with generated notes (code reading, not measured)', async () => {
  const fake = await runRelease('prod/acme/v0.1.0', []);
  assert.equal(fake.calls.createRelease.length, 1);
  const payload = fake.calls.createRelease[0];
  assert.equal(payload.make_latest, 'false');
  assert.equal(payload.generate_release_notes, true);
});

test('C5: plain-tag draft is published with draft and make_latest only (R-A2 (b))', async () => {
  // Measured: one Release for the tag. Exact key set of the payload: code reading, not measured.
  const draft = rel('v3.1.0', { draft: true, id: 777 });
  const fake = await runRelease('v3.1.0', [draft]);
  assert.equal(fake.calls.createRelease.length, 0);
  assert.equal(fake.calls.updateRelease.length, 1);
  const payload = fake.calls.updateRelease[0];
  assert.equal(payload.release_id, 777);
  assert.equal(payload.draft, false);
  assert.equal(payload.make_latest, 'true');
  assert.deepEqual(Object.keys(payload).sort(), ['draft', 'make_latest', 'owner', 'release_id', 'repo']);
});

test('C6: prod draft is published with draft false and make_latest false only', async () => {
  // Expectation from code reading, not measured.
  const draft = rel('prod/acme/v0.1.0', { draft: true, id: 778 });
  const fake = await runRelease('prod/acme/v0.1.0', [draft]);
  assert.equal(fake.calls.createRelease.length, 0);
  assert.equal(fake.calls.updateRelease.length, 1);
  const payload = fake.calls.updateRelease[0];
  assert.equal(payload.release_id, 778);
  assert.equal(payload.draft, false);
  assert.equal(payload.make_latest, 'false');
  assert.deepEqual(Object.keys(payload).sort(), ['draft', 'make_latest', 'owner', 'release_id', 'repo']);
});

test('C7: a published Release of the tag is skipped (R-A2 (c): release_action skip)', async () => {
  const fake = await runRelease('v3.2.0', [rel('v3.2.0')]);
  assert.equal(fake.calls.createRelease.length, 0);
  assert.equal(fake.calls.updateRelease.length, 0);
  assert.equal(fake.outputs.action, 'skip');
});

test('C8: reassert sets Latest on the highest published version (R-A2 (e))', async () => {
  // Measured: Latest returned to the highest version. Call shape: code reading, not measured.
  const list = [rel('v3.2.0', { id: 321 }), rel('v3.1.5', { id: 315 })];
  const fake = await runReassert('v3.1.5', list, { releaseAction: 'skip' });
  assert.equal(fake.calls.updateRelease.length, 1);
  const payload = fake.calls.updateRelease[0];
  assert.equal(payload.release_id, 321);
  assert.equal(payload.make_latest, 'true');
  assert.equal(fake.failures.length, 0);
});

test('C9: reassert never gives Latest to a prod Release (code reading, not measured)', async () => {
  const list = [rel('prod/acme/v1.0.0', { id: 100 }), rel('v3.8.0', { id: 380 })];
  const fake = await runReassert('prod/acme/v1.0.0', list, { releaseAction: 'skip' });
  assert.equal(fake.calls.updateRelease.length, 1);
  assert.equal(fake.calls.updateRelease[0].release_id, 380);
  assert.equal(fake.calls.updateRelease[0].make_latest, 'true');
});

test('C10: release step outputs for create, publish and skip (create: R-A2 (j))', async () => {
  // Create measured; publish, skip and the prerelease output: code reading, not measured.
  const created = await runRelease('v3.0.0', []);
  assert.equal(created.outputs.action, 'create');
  assert.equal(created.outputs.release_id, '9001');
  assert.equal(created.outputs.prerelease, 'false');

  const published = await runRelease('v3.1.0', [rel('v3.1.0', { draft: true, id: 777 })]);
  assert.equal(published.outputs.action, 'publish');
  assert.equal(published.outputs.release_id, '777');
  assert.equal(published.outputs.prerelease, 'false');

  const skipped = await runRelease('v3.2.0', [rel('v3.2.0')]);
  assert.equal(skipped.outputs.action, 'skip');
});

// ---------------------------------------------------------------------------
// Group W: defects of run.js (U6a)
// ---------------------------------------------------------------------------

test('W1: reassert after create/publish gives Latest to this Release ' +
  'when the list holds it as a stale draft', async () => {
  const list = [rel('v3.9.0', { draft: true, id: 900 }), rel('v3.8.0', { id: 380 })];
  const fake = await runReassert('v3.9.0', list, {
    releaseAction: 'publish',
    releaseId: '900',
    releasePrerelease: 'false',
  });
  assert.equal(fake.calls.updateRelease.length, 1);
  assert.equal(fake.calls.updateRelease[0].release_id, 900);
  assert.equal(fake.calls.updateRelease[0].make_latest, 'true');
});

test('W2: reassert with the get-by-tag fallback gives Latest to this Release ' +
  'when the list holds it as a stale draft', async () => {
  const list = [rel('v3.9.0', { draft: true, id: 900 }), rel('v3.8.0', { id: 380 })];
  const published = rel('v3.9.0', { id: 900 });
  const fake = await runReassert('v3.9.0', list, { releaseAction: 'skip' }, { tagLookup: published });
  assert.equal(fake.calls.updateRelease.length, 1);
  assert.equal(fake.calls.updateRelease[0].release_id, 900);
  assert.equal(fake.calls.updateRelease[0].make_latest, 'true');
});
