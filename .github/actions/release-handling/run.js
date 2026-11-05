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

async function release({ github, context, core, tag }) {
  const { owner, repo } = context.repo;
  const res = decideAction(tag, await listReleases(github, owner, repo));
  if (res.action === 'fail') {
    core.setFailed(res.reason);
  } else if (res.action === 'skip') {
    core.info(`A published Release of ${tag} exists; nothing to create`);
  } else if (res.action === 'create') {
    const p = parseTag(tag);
    await github.rest.repos.createRelease({
      owner,
      repo,
      tag_name: tag,
      name: p.cls === 'prod' ? `[${p.id}] v${p.version}` : tag,
      draft: false,
      prerelease: p.cls === 'v-suffix',
      generate_release_notes: true,
      make_latest: String(res.makeLatest),
    });
  } else {
    // Publishing a draft sets the draft flag and make_latest only; the draft body is kept.
    await github.rest.repos.updateRelease({
      owner,
      repo,
      release_id: res.releaseId,
      draft: false,
      make_latest: String(res.makeLatest),
    });
  }
}

// Runs on every push, also after a failed build; the list is fetched again here.
async function reassert({ github, context, core }) {
  const { owner, repo } = context.repo;
  const list = await listReleases(github, owner, repo);
  const target = latestTarget(list);
  if (target === null) {
    core.info('No Latest target');
    return;
  }
  const found = list.find((r) => r.tag_name === target && !r.draft);
  await github.rest.repos.updateRelease({ owner, repo, release_id: found.id, make_latest: 'true' });
}

module.exports = async function run({ github, context, core, mode, tag }) {
  if (mode === 'parse') {
    await parse(core, tag);
  } else if (mode === 'release') {
    await release({ github, context, core, tag });
  } else if (mode === 'reassert') {
    await reassert({ github, context, core });
  } else {
    core.setFailed(`unknown mode '${mode}'`);
  }
};
