#!/usr/bin/env bash

# Internal script to build dependencies for a macOS Drake wheel. This is a
# wrapper over the generic script that performs additional environment
# preparation.

set -eu -o pipefail

readonly resource_root="$(
    cd "$(dirname "${BASH_SOURCE}")" && \
    realpath ..
)"

