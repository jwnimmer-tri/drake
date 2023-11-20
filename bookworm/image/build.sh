#!/bin/bash

set -eu -o pipefail

mkdir /tmp/drake-build
cd /tmp/drake-build

# Store downloads in the build cache to speed up rebuilds.
export BAZELISK_HOME=/var/cache/bazel/bazelisk

# Add wheel-specific bazel options.
cat > /etc/bazel.bazelrc << EOF
build --disk_cache=/var/cache/bazel/disk_cache
build --repository_cache=/var/cache/bazel/repository_cache
EOF

# Install Drake using our wheel-build-specific Python interpreter.
cmake /image/drake \
    -DCMAKE_INSTALL_PREFIX=/opt/drake \
    -DPython_EXECUTABLE=/usr/bin/python3
make install
