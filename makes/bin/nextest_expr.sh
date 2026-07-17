#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
if [ -z "${mode}" ]; then
  echo "usage: nextest_expr.sh <fast|slow>" >&2
  exit 2
fi

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
roster_path="${workspace_root}/configs/rust/nextest-slow-roster.txt"
legacy_expr='test(/::slow__/)'

roster_regex() {
  if [ ! -f "${roster_path}" ]; then
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

composed_slow_expr() {
  local regex
  regex="$(roster_regex)"
  if [ -n "${regex}" ]; then
    printf '%s\n' "${legacy_expr} or test(/^(?:${regex})$/)"
  else
    printf '%s\n' "${legacy_expr}"
  fi
}

case "${mode}" in
  fast)
    slow_expr="$(composed_slow_expr)"
    if [ "${slow_expr}" = "${legacy_expr}" ]; then
      printf '%s\n' "not ${slow_expr}"
    else
      printf '%s\n' "not (${slow_expr})"
    fi
    ;;
  slow)
    composed_slow_expr
    ;;
  *)
    echo "unknown mode: ${mode}" >&2
    exit 2
    ;;
esac
