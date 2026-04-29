# Required Status Checks

- Owner: `build-and-release`
- Source of truth: workflow and job names in `.github/workflows/*.yml`

Required branch-protection checks for `main`:

- `std / standard` (from workflow `bijux-std`)
- `std / report` (from workflow `bijux-std`)

Notes:

- Keep required checks limited to jobs that run on every pull request to avoid merge deadlocks from path-filtered workflows.
- If workflow or job names change, update this file and `.github/rulesets/main-branch-protection.json` in the same change.
- Main branch bypass is not allowed; changes must merge through pull requests.
- For `pull_request_target` workflows, GitHub evaluates workflow content from the base branch, not from the pull request head branch.
