# Required Status Checks

- Owner: `build-and-release`
- Source of truth: workflow and job names in `.github/workflows/*.yml`

Required branch-protection checks for `main`:

- `policy / github` (from workflow `policy / github`)
- `policy / pr approval` (from workflow `policy / pr approval`)
- `std / standard` (from workflow `bijux-std`)
- `std / report` (from workflow `bijux-std`)

Notes:

- Treat these four checks as the baseline for all governed repos in the unified contract.
- Add repository-specific checks on top of this baseline only when they run on every pull request for that repository.
- If workflow or job names change, update this file and `.github/rulesets/main-branch-protection.json` in the same change.
- Main branch bypass is not allowed; changes must merge through pull requests.
- For `pull_request_target` workflows, GitHub evaluates workflow content from the base branch, not from the pull request head branch.
