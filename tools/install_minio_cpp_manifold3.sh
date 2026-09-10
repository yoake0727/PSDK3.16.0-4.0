#!/usr/bin/env bash
set -euo pipefail
prefix="${1:-$PWD/third_party/aarch64}"
src="${MINIOCPP_SOURCE_DIR:?set MINIOCPP_SOURCE_DIR to a minio-cpp checkout}"
cmake -S "$src" -B "$PWD/build/minio-cpp" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DMINIO_CPP_ENABLE_RDMA=OFF
cmake --build "$PWD/build/minio-cpp" --parallel "${MINIOCPP_JOBS:-2}"
cmake --install "$PWD/build/minio-cpp"
test -f "$prefix/lib/libminiocpp.a"
echo "minio-cpp installed under $prefix"
