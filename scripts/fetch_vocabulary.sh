#!/usr/bin/env bash
# Extracts the ORB vocabulary shipped with the ORB-SLAM3 submodule.
#
# It lives in the repo as a 42MB tarball and expands to ~145MB of text, which
# is why it is not extracted at clone time. The SAME extracted file must be
# used for building a map and for localizing against it: ORB-SLAM3 stores its
# checksum in the map and rejects a mismatch.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vocabulary_dir="$repo_root/third_party/ORB_SLAM3/Vocabulary"
archive="$vocabulary_dir/ORBvoc.txt.tar.gz"
target="$vocabulary_dir/ORBvoc.txt"

if [[ -f "$target" ]]; then
    echo "Vocabulary already extracted: $target"
    exit 0
fi

if [[ ! -f "$archive" ]]; then
    echo "error: $archive not found. Run: git submodule update --init --recursive" >&2
    exit 1
fi

echo "Extracting ORB vocabulary (this takes a moment)..."
tar -xzf "$archive" -C "$vocabulary_dir"
echo "Done: $target"
