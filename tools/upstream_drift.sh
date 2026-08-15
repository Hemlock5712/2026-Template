#!/usr/bin/env bash
# Diff our vendored copies of WPILib classes (src/main/java/org/wpilib/**) against upstream
# allwpilib at the alpha tag build.gradle is pinned to. Run this when bumping the pin: the
# compiler catches renamed/removed APIs, but a silently-changed vendored file compiles fine.
# Needs the `gh` CLI. Usage: tools/upstream_drift.sh [tag]
set -euo pipefail
cd "$(dirname "$0")/.."

tag="${1:-v$(grep -o 'GradleRIO" version "[^"]*' build.gradle | cut -d'"' -f3)}"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
echo "Comparing against allwpilib $tag"

for local in $(find src/main/java/org/wpilib -name '*.java'); do
  rel=${local#src/main/java/}
  echo "===== $rel"
  # --strip-trailing-cr: our working copies are CRLF, upstream is LF.
  if gh api "repos/wpilibsuite/allwpilib/contents/wpilibj/src/main/java/$rel?ref=$tag" \
      --jq .content 2>/dev/null | base64 -d > "$tmp/up.java"; then
    diff -u --strip-trailing-cr "$tmp/up.java" "$local" || true
  else
    echo "  NOT FOUND upstream at $tag - moved, renamed, or deleted. Check before bumping."
  fi
done
