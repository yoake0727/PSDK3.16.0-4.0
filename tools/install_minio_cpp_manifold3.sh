#!/usr/bin/env bash
set -euo pipefail
prefix="${1:-$PWD/third_party/aarch64}"
src="${MINIOCPP_SOURCE_DIR:?set MINIOCPP_SOURCE_DIR to a minio-cpp checkout}"

patch_file="$PWD/tools/zstd_alias_patch.cmake"
httplib_src="/home/dji/cpp-httplib-old/cpp-httplib-0.11.4"

# OpenSSL 3 装到用户目录
openssl3_prefix="/home/dji/openssl-3"

rm -rf "$PWD/build/minio-cpp"

cmake -S "$src" -B "$PWD/build/minio-cpp" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_TESTING=OFF \
  -DMINIO_CPP_ENABLE_RDMA=OFF \
  -DCMAKE_PROJECT_INCLUDE="$patch_file" \
  -DFETCHCONTENT_SOURCE_DIR_CPP-HTTPLIB="$httplib_src" \
  -DOPENSSL_ROOT_DIR="$openssl3_prefix" \
  -DOPENSSL_INCLUDE_DIR="$openssl3_prefix/include" \
  -DOPENSSL_CRYPTO_LIBRARY="$openssl3_prefix/lib/libcrypto.so" \
  -DOPENSSL_SSL_LIBRARY="$openssl3_prefix/lib/libssl.so"

cmake --build "$PWD/build/minio-cpp" --parallel "${MINIOCPP_JOBS:-2}"
cmake --install "$PWD/build/minio-cpp"
test -f "$prefix/lib/libminiocpp.a"
echo "minio-cpp installed under $prefix"