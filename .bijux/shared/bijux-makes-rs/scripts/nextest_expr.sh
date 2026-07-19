#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
if [[ "${mode}" != "fast" && "${mode}" != "slow" ]]; then
  echo "usage: nextest_expr.sh <fast|slow>" >&2
  exit 2
fi

workspace_root="${PROJECT_ROOT:-$(git rev-parse --show-toplevel)}"
roster_path="${NEXTEST_SLOW_ROSTER:-${workspace_root}/configs/rust/nextest-slow-roster.txt}"
name_expr="${NEXTEST_SLOW_NAME_EXPR:-test(/(^|::)slow_/)}"

roster_regex() {
  if [[ ! -f "${roster_path}" ]]; then
    return 0
  fi
  perl -ne '
    next if /^\s*(?:#|$)/;
    s/^\s+|\s+$//g;
    next if $_ eq q{};
    chomp;
    push @names, quotemeta($_);
    END {
      my %seen;
      @names = grep { !$seen{$_}++ } @names;
      print join(q{|}, @names);
    }
  ' "${roster_path}"
}

slow_expr() {
  local roster
  roster="$(roster_regex)"
  if [[ -n "${roster}" ]]; then
    printf '%s\n' "${name_expr} or test(/^(?:${roster})$/)"
  else
    printf '%s\n' "${name_expr}"
  fi
}

if [[ "${mode}" == "slow" ]]; then
  slow_expr
else
  expression="$(slow_expr)"
  printf 'not (%s)\n' "${expression}"
fi
