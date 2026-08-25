#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_prefix="${repo_root}/third_party/aarch64"
build_root="${repo_root}/build/static-deps"
jobs="${STATIC_DEPS_JOBS:-2}"

: "${FFMPEG_SOURCE_DIR:?set FFMPEG_SOURCE_DIR to an existing FFmpeg source tree}"
: "${MOSQUITTO_SOURCE_DIR:?set MOSQUITTO_SOURCE_DIR to an existing Mosquitto source tree}"
zlib_static_library="${ZLIB_STATIC_LIBRARY:-}"

if test -z "$zlib_static_library"; then
    for candidate in \
        /usr/lib/aarch64-linux-gnu/libz.a \
        /lib/aarch64-linux-gnu/libz.a \
        /usr/lib/libz.a \
        /lib/libz.a; do
        if test -f "$candidate"; then
            zlib_static_library="$candidate"
            break
        fi
    done
fi

if test -z "$zlib_static_library" || test ! -f "$zlib_static_library"; then
    echo "static zlib archive not found; set ZLIB_STATIC_LIBRARY to an existing libz.a" >&2
    exit 1
fi

for required_tool in cmake make gcc g++ ar file; do
    command -v "$required_tool" >/dev/null 2>&1 || {
        echo "required tool not found: $required_tool" >&2
        exit 1
    }
done

for source_dir in \
    "$FFMPEG_SOURCE_DIR" \
    "$MOSQUITTO_SOURCE_DIR"; do
    test -d "$source_dir" || {
        echo "source directory not found: $source_dir" >&2
        exit 1
    }
done

test -f "${FFMPEG_SOURCE_DIR}/configure" || {
    echo "FFmpeg configure not found: ${FFMPEG_SOURCE_DIR}/configure" >&2
    exit 1
}
ffmpeg_helper_scripts=("${FFMPEG_SOURCE_DIR}"/ffbuild/*.sh)
test -f "${ffmpeg_helper_scripts[0]}" || {
    echo "FFmpeg ffbuild helper scripts not found: ${FFMPEG_SOURCE_DIR}/ffbuild/*.sh" >&2
    exit 1
}
# FFmpeg-generated Makefiles execute these helpers directly. Source archives
# copied from non-POSIX filesystems can lose their executable permission bits.
chmod u+x "${ffmpeg_helper_scripts[@]}"

test -f "${MOSQUITTO_SOURCE_DIR}/CMakeLists.txt" || {
    echo "Mosquitto CMakeLists.txt not found" >&2
    exit 1
}
case "$jobs" in
    ''|*[!0-9]*|0)
        echo "STATIC_DEPS_JOBS must be a positive integer" >&2
        exit 1
        ;;
esac

safe_recreate_dir()
{
    target_dir="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
    case "$target_dir" in
        "${repo_root}/build/static-deps"|"${repo_root}/third_party/aarch64"|"${repo_root}/build/static-deps/"*)
            ;;
        *)
            echo "refusing to recreate directory outside approved roots: $target_dir" >&2
            exit 1
            ;;
    esac
    cmake -E remove_directory "$target_dir"
    cmake -E make_directory "$target_dir"
}

safe_recreate_dir "$build_root"
safe_recreate_dir "$install_prefix"
cmake -E make_directory "${install_prefix}/lib"
cmake -E copy "$zlib_static_library" "${install_prefix}/lib/libz.a"

ffmpeg_build="${build_root}/ffmpeg"
mosquitto_build="${build_root}/mosquitto"
cmake -E make_directory "$ffmpeg_build"

echo "Configuring minimal static FFmpeg"
(
    cd "$ffmpeg_build"
    bash "${FFMPEG_SOURCE_DIR}/configure" \
        --prefix="$install_prefix" \
        --arch=aarch64 \
        --target-os=linux \
        --cc=gcc \
        --cxx=g++ \
        --enable-static \
        --disable-shared \
        --enable-pic \
        --disable-programs \
        --disable-doc \
        --disable-debug \
        --disable-autodetect \
        --disable-everything \
        --disable-avdevice \
        --disable-avfilter \
        --enable-swscale \
        --disable-swresample \
        --enable-avcodec \
        --enable-avformat \
        --enable-avutil \
        --enable-network \
        --enable-zlib \
        --enable-pthreads \
        --enable-decoder=h264 \
        --enable-encoder=mjpeg \
        --enable-parser=h264 \
        --enable-muxer=rtsp \
        --enable-muxer=rtp \
        --enable-protocol=tcp \
        --enable-protocol=udp \
        --enable-protocol=rtp
    make -j"$jobs"
    make install
)

echo "Configuring static Mosquitto client library without TLS"
cmake -S "$MOSQUITTO_SOURCE_DIR" -B "$mosquitto_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$install_prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DWITH_STATIC_LIBRARIES=ON \
    -DWITH_SHARED_LIBRARIES=OFF \
    -DWITH_TLS=OFF \
    -DWITH_BROKER=OFF \
    -DWITH_APPS=OFF \
    -DWITH_DOCS=OFF \
    -DDOCUMENTATION=OFF \
    -DWITH_TESTS=OFF \
    -DWITH_PIC=ON
cmake --build "$mosquitto_build" --parallel "$jobs"
cmake --install "$mosquitto_build"

if test ! -f "${install_prefix}/lib/libmosquitto.a"; then
    for candidate in \
        "${mosquitto_build}/lib/libmosquitto_static.a" \
        "${mosquitto_build}/lib/libmosquitto.a"; do
        if test -f "$candidate"; then
            cmake -E copy "$candidate" "${install_prefix}/lib/libmosquitto.a"
            break
        fi
    done
fi
if test ! -f "${install_prefix}/include/mosquitto.h"; then
    cmake -E copy "${MOSQUITTO_SOURCE_DIR}/include/mosquitto.h" \
        "${install_prefix}/include/mosquitto.h"
fi

required_outputs=(
    "${install_prefix}/include/mosquitto.h"
    "${install_prefix}/include/libavcodec/avcodec.h"
    "${install_prefix}/include/libavformat/avformat.h"
    "${install_prefix}/include/libavutil/avutil.h"
    "${install_prefix}/include/libswscale/swscale.h"
    "${install_prefix}/lib/libmosquitto.a"
    "${install_prefix}/lib/libz.a"
    "${install_prefix}/lib/libavformat.a"
    "${install_prefix}/lib/libavcodec.a"
    "${install_prefix}/lib/libswscale.a"
    "${install_prefix}/lib/libavutil.a"
)

for output_file in "${required_outputs[@]}"; do
    test -f "$output_file" || {
        echo "required static dependency output missing: $output_file" >&2
        exit 1
    }
done

file "${install_prefix}/lib/libmosquitto.a"
file "${install_prefix}/lib/libz.a"
file "${install_prefix}/lib/libavformat.a"
file "${install_prefix}/lib/libswscale.a"
ar t "${install_prefix}/lib/libmosquitto.a" >/dev/null
ar t "${install_prefix}/lib/libz.a" >/dev/null
ar t "${install_prefix}/lib/libavformat.a" >/dev/null
ar t "${install_prefix}/lib/libswscale.a" >/dev/null

echo "Static dependencies installed successfully: $install_prefix"
