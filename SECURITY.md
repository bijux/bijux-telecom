# Security Policy

Report suspected vulnerabilities privately. Do not open a public issue with
exploit details, credentials, private data, or an unpatched reproduction.

## Supported Releases

| Surface | Support status |
| --- | --- |
| latest official tagged release and its repository-published artifacts | receives security assessment and fixes when a report is confirmed and remediation is feasible |
| older tagged releases | not ordinarily patched; reporters may be asked to verify the latest release |
| `main`, development branches, and local source builds | reports are accepted, but these are not supported release artifacts |
| repository-only maintainer crates and automation | assessed when they can compromise CI, release authority, or published artifacts |
| third-party receivers, data providers, container runtimes, and external services | maintained by their owners unless the vulnerability is caused by a Bijux integration defect |

An official artifact is produced by this repository's release workflows from a
tag. A workspace version or untagged source build does not establish release
support.

## Report Privately

Preferred channel:

- [GitHub private vulnerability reporting](https://github.com/bijux/bijux-gnss/security/advisories/new)

Fallback:

- [bijan@bijux.io](mailto:bijan@bijux.io)

Include enough information to reproduce and assess the issue:

- affected crate, command, version, and installation method;
- operating system, architecture, and relevant feature flags;
- input type and provenance, using a minimal synthetic sample when possible;
- reproduction steps or proof of concept;
- expected and observed security boundary;
- impact, required privileges, and whether user interaction is required; and
- whether secrets, private captures, or location data may have been exposed.

Remove live credentials and personal data. Share sensitive or large evidence
only through a channel agreed during private triage.

## Repository Trust Boundaries

### Inputs and datasets

GNSS commands parse local captures, configuration, sidecars, navigation
products, and persisted run evidence. Validation establishes the documented
format and repository contracts; it does not make untrusted input safe for
unbounded resource use.

Path traversal, unsafe writes outside the selected artifact root, parser
resource exhaustion, or a provenance-validation bypass caused by
repository-owned code is in scope. Incorrect or malicious measurements that
remain within the documented data model are a data-quality issue unless they
cross a claimed security boundary.

### Command execution and artifacts

The `bijux gnss` command runs with the invoking user's filesystem and process
privileges. It is not a sandbox. Artifact-root checks, typed configuration,
hashes, and manifests protect repository-owned execution and evidence
contracts; they do not isolate the host from trusted commands or third-party
libraries.

A defect that bypasses an enforced path, configuration, artifact-integrity, or
release-provenance boundary is in scope.

### Scientific integrity

Accuracy, uncertainty, integrity, and refusal behavior are product contracts,
but an ordinary numerical defect is not automatically a security
vulnerability. Report privately when a defect can systematically conceal
invalid evidence, bypass an enforced integrity decision, or create a credible
safety or supply-chain impact. Use regular issues for non-sensitive scientific
or numerical defects.

### Build and publication

Release workflows, dependency handling, crate packages, container images,
checksums, generated provenance, and credential boundaries are in scope when a
defect can compromise an official artifact or publication authority.

## In-Scope Examples

- path traversal or unauthorized writes outside an owned artifact root;
- malformed input that causes exploitable memory-safety behavior or practical
  resource exhaustion;
- enforced configuration, integrity, checksum, or provenance validation
  bypass;
- secret or private-data disclosure through logs, reports, packages, images,
  or release artifacts;
- supply-chain or workflow defects that allow unauthorized publication; and
- materially false isolation or security-boundary claims caused by
  repository-owned code.

## Out of Scope

- unsupported versions or platforms unless the issue also affects a supported
  release;
- malicious or inaccurate GNSS data without a repository-owned security
  boundary failure;
- behavior of third-party tools or services within their documented
  privileges;
- social engineering, physical access, or compromised maintainer accounts
  without a repository defect;
- availability testing that creates excessive traffic, resource exhaustion, or
  service disruption; and
- version labels or unreleased source state without security impact.

## Coordinated Handling

The maintainer will assess reproducibility, affected releases, impact, and the
appropriate remediation and disclosure route. The project is maintained on a
best-effort basis and does not guarantee a response or remediation deadline.
Please allow private triage before public disclosure.

When appropriate, remediation will include affected code, regression tests,
release artifacts, and a GitHub security advisory. A report may be closed as
out of scope, not reproducible, or already addressed, with the reasoning shared
privately.

There is no public bug bounty program. Non-security defects belong in regular
GitHub issues.
