'use strict';

const { parseTag, decideAction, latestTarget } = require('./release-handling.js');

async function listReleases(github, owner, repo) {
  return github.paginate(github.rest.repos.listReleases, { owner, repo, per_page: 100 });
}

async function parse(core, tag) {
  const p = parseTag(tag);
  core.setOutput('cls', p.cls);
  if (p.cls === 'invalid') {
    core.setFailed(p.reason);
  }
}

function setReleaseOutputs(core, action, data) {
  core.setOutput('action', action);
  core.setOutput('release_id', data ? String(data.id) : '');
  core.setOutput('prerelease', data ? String(data.prerelease) : '');
}

async function release({ github, context, core, tag }) {
  const { owner, repo } = context.repo;
  const res = decideAction(tag, await listReleases(github, owner, repo));
  if (res.action === 'fail') {
    setReleaseOutputs(core, res.action, null);
    core.setFailed(res.reason);
  } else if (res.action === 'skip') {
    setReleaseOutputs(core, res.action, null);
    core.info(`A published Release of ${tag} exists; nothing to create`);
  } else if (res.action === 'create') {
    const p = parseTag(tag);
    const { data } = await github.rest.repos.createRelease({
      owner,
      repo,
      tag_name: tag,
      name: p.cls === 'prod' ? `[${p.id}] v${p.version}` : tag,
      draft: false,
      prerelease: p.cls === 'v-suffix',
      generate_release_notes: true,
      make_latest: String(res.makeLatest),
    });
    setReleaseOutputs(core, res.action, data);
  } else {
    // Publishing a draft sets the draft flag and make_latest only; the draft body is kept.
    const { data } = await github.rest.repos.updateRelease({
      owner,
      repo,
      release_id: res.releaseId,
      draft: false,
      make_latest: String(res.makeLatest),
    });
    setReleaseOutputs(core, res.action, data);
  }
}

// Runs on every push, also after a failed build; the list is fetched again here.
// The list lags a few seconds after a create, so this run's Release is added to it explicitly.
async function reassert({ github, context, core, tag, releaseAction, releaseId, releasePrerelease }) {
  const { owner, repo } = context.repo;
  const list = await listReleases(github, owner, repo);
  if (releaseAction === 'create' || releaseAction === 'publish') {
    if (releaseId === '') {
      core.setFailed(`the release job reported ${releaseAction} without a Release id`);
      return;
    }
    const made = { id: Number(releaseId), tag_name: tag, draft: false, prerelease: releasePrerelease === 'true' };
    if (!list.some((r) => r.id === made.id)) {
      list.push(made);
    }
  } else {
    // Skip, fail or no release job: fall back to fetching the tag's Release directly.
    try {
      const { data } = await github.rest.repos.getReleaseByTag({ owner, repo, tag });
      if (data && !data.draft && !list.some((r) => r.id === data.id)) {
        list.push(data);
      }
    } catch (error) {
      core.info(`No published Release for ${tag}: ${error.message}`);
    }
  }
  const target = latestTarget(list);
  if (target === null) {
    core.info('No Latest target');
    return;
  }
  const found = list.find((r) => r.tag_name === target && !r.draft);
  await github.rest.repos.updateRelease({ owner, repo, release_id: found.id, make_latest: 'true' });
}

module.exports = async function run({ github, context, core, mode, tag, releaseAction, releaseId, releasePrerelease }) {
  if (mode === 'parse') {
    await parse(core, tag);
  } else if (mode === 'release') {
    await release({ github, context, core, tag });
  } else if (mode === 'reassert') {
    await reassert({ github, context, core, tag, releaseAction, releaseId, releasePrerelease });
  } else {
    core.setFailed(`unknown mode '${mode}'`);
  }
};
