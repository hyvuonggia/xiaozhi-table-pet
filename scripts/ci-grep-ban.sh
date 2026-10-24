#!/bin/sh
# CI ban (Oracle binding): no portMAX_DELAY or ESP_ERROR_CHECK in main/.
# Rationale (SPEC §5/§6): portMAX_DELAY = hang forever on error;
# ESP_ERROR_CHECK = abort()/panic = reboot loop. Runtime errors must be
# handled softly. A line carrying the token ALLOWLIST is exempt (use
# sparingly, with a comment explaining why it is safe).
# Usage: sh scripts/ci-grep-ban.sh
set -eu
hits=""
for pat in 'portMAX_DELAY' 'ESP_ERROR_CHECK'; do
  found=$(grep -rn --include='*.c' --include='*.h' -E "$pat" main/ 2>/dev/null | grep -v 'ALLOWLIST' || true)
  if [ -n "$found" ]; then
    printf 'BANNED %s:\n%s\n' "$pat" "$found"
    hits=1
  fi
done
if [ -n "$hits" ]; then
  echo "ci-grep-ban: FAIL" >&2
  exit 1
fi
echo "ci-grep-ban: OK"
