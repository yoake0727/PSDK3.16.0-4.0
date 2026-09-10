# 妙算 3 配置 MinIO C++ SDK

本工程使用 MinIO 官方 `minio-cpp` SDK，由 SDK 完成 AWS Signature V4，不再包含手写或伪造签名。

## 1. 安装构建依赖

在妙算 3 原生 Ubuntu AArch64 环境执行：

```bash
sudo apt update
sudo apt install -y git build-essential pkg-config cmake \
  libssl-dev zlib1g-dev nlohmann-json3-dev libpugixml-dev
cmake --version
g++ --version
```

要求 CMake 不低于 3.13.4，并且编译器支持 C++17。若系统 CMake 过旧，需要先升级 CMake，不能降低本工程的 C++ 标准。

## 2. 获取并安装官方 SDK

```bash
cd /home/amov
git clone https://github.com/minio/minio-cpp.git
cd /path/to/PSDK3.16.0-4.3-9.3-dingwei
export MINIOCPP_SOURCE_DIR=/home/amov/minio-cpp
chmod +x tools/install_minio_cpp_manifold3.sh
./tools/install_minio_cpp_manifold3.sh "$PWD/third_party/aarch64"
```

脚本关闭 RDMA、构建静态库，并将 SDK、CMake 配置和它构建的依赖安装到 `third_party/aarch64`。

## 3. 配置运行凭据

账号密码已按项目约定统一写入 `/home/dji/PSDK3.16.0-4.3-9.3-dingwei/src/app_json/app_config.yaml`。请将实际值写入：

```yaml
minio.access_key: 实际访问密钥
minio.secret_key: 实际密钥
```

地址、桶、目录和账号密码统一在 `/home/dji/PSDK3.16.0-4.3-9.3-dingwei/src/app_json/app_config.yaml` 配置。当前代码按 HTTP 连接；启用 HTTPS 前需要同步增加 `minio.secure` 配置处理及 CA 证书部署。

## 4. 编译与验证

```bash
cmake -S src -B build/manifold3 \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/aarch64"
cmake --build build/manifold3 --parallel 2
```

运行前确认 MinIO 可达且桶已经存在：

```bash
nc -vz 192.168.144.125 9000
```

启动工程后检查三项：MinIO 中出现 JPEG；成功反馈含 `fileUrl`；同一帧的 `locations` 消息只在上传成功后出现。临时使用错误密钥验证失败路径，此时必须收到失败反馈且不能收到该帧坐标。
