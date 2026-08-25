# 妙算3第三方静态依赖构建说明

本工具在妙算3AArch64环境中构建应用所需的Mosquitto和最小FFmpeg静态库。
它不会下载源码、不会执行 `sudo`，也不会处理MediaMTX。

OpenCV不由本工具构建或修改，应用继续使用妙算3系统OpenCV。

## 1. 准备源码

分别准备固定版本的FFmpeg和Mosquitto源码目录。建议选择与当前妙算3已验证版本
相同或API兼容的版本，并保存源码版本号或Git提交号。

不要把系统发行版的完整FFmpeg静态库直接复制到仓库，否则会重新引入
`librsvg`、GLib和大量无关编解码依赖。

FFmpeg 4.2.x的RTP muxer会静态引用MJPEG公共Huffman表，因此脚本同时启用内置
MJPEG encoder来满足链接依赖；H30T输出仍然是H.264，不会转换为MJPEG。

## 2. 设置环境变量

```bash
cd ~/Payload-SDK-3.16.0

export FFMPEG_SOURCE_DIR="$HOME/src/ffmpeg"
export MOSQUITTO_SOURCE_DIR="$HOME/src/mosquitto"
export STATIC_DEPS_JOBS=2
```

两个源码目录必须已经存在。脚本不会自动联网下载。

## 3. 执行构建

```bash
bash tools/build_manifold3_static_deps.sh
```

脚本会为FFmpeg源码树中的 `ffbuild/*.sh` 恢复当前用户的执行权限。FFmpeg生成的
Makefile会直接调用这些辅助脚本；从Windows文件系统或不保留Unix权限的压缩包
复制源码时，执行位可能丢失并导致 `Permission denied`。

脚本将重建以下两个仓库内目录：

```text
build/static-deps
third_party/aarch64
```

不会删除源码目录或仓库外目录。

## 4. 生成内容

```text
third_party/aarch64/
├── include/
│   ├── libavcodec/
│   ├── libavformat/
│   ├── libavutil/
│   ├── libswscale/
│   └── mosquitto.h
└── lib/
    ├── libavcodec.a
    ├── libavformat.a
    ├── libswscale.a
    ├── libavutil.a
    └── libmosquitto.a
```

当前Mosquitto静态库关闭TLS，只支持普通MQTT TCP连接及用户名密码认证。
如果broker要求MQTTS，不能使用本构建，需要另外引入静态OpenSSL和证书配置。

OpenCV保留当前系统动态链接、HighGUI、DNN及CUDA/NVIDIA相关能力。

## 5. 重建应用

静态依赖生成后必须清理旧CMake缓存：

```bash
cd ~/Payload-SDK-3.16.0
rm -rf build
mkdir build
cd build
cmake ..
make -j2
```

## 6. 检查ELF依赖

```bash
ldd bin/dji_sdk_demo_on_manifold3_cxx
readelf -d bin/dji_sdk_demo_on_manifold3_cxx | grep NEEDED
```

输出中不应再出现：

```text
libmosquitto.so
libavcodec.so
libavformat.so
libavutil.so
libswscale.so
```

OpenCV、系统、Qt/X11、CUDA/NVIDIA、DJI平台库仍可保持动态。

## 7. DPK验证

确认已有MediaMTX文件仍在：

```bash
ls -l \
  samples/sample_c++/platform/linux/manifold3/application/app_json/runtime/mediamtx
```

重新打包、安装，并验证MQTT、H30T双路RTSP、OpenCV和退出清理。
