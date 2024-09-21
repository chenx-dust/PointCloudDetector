## 构建指南

### 安装依赖

本项目是跨平台的[^1]，但为保证可复现性，我们以 Ubuntu 22.04 作为参考部署环境。由于本项目利用了源内版本的 Open3D 不具备的功能，必须手动编译安装 Open3D （见下一节）。

```bash
sudo apt install libeigen3-dev libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev libboost-dev libtbb-dev libpaho-mqtt-dev libpaho-mqttpp-dev
```

### 编译安装 [Open3D](https://github.com/isl-org/Open3D)

推荐编译选项已写入下方，减少无用组件编译，使用系统 `fmt` 解决 Ubuntu 22.04 的部署问题，采用 Release 编译配置获得更好的性能。

需要注意的是，Open3D 会在构建时下载一些依赖，请确保网络通畅。

```bash
git clone -b v0.18.0 https://github.com/isl-org/Open3D.git && cd Open3D
util/install_deps_ubuntu.sh # Password needed
mkdir build && cd build
cmake -DBUNDLE_OPEN3D_ML=OFF -DBUILD_UNIT_TESTS=OFF -DWITH_OPENMP=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_EXAMPLES=OFF -DUSE_SYSTEM_FMT=ON -DGLIBCXX_USE_CXX11_ABI=1 -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 项目构建

```bash
git clone https://github.com/chenx-dust/PointCloudDetector.git
cd PointCloudDetector
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j $(nproc)
```
### 性能优化提示

- 使用较高的优化等级，本项目对浮点运算精度要求不高，可较为放心地启用优化
- 实际部署请使用 Release 配置
- 使用 Intel oneAPI ICX 编译器编译，在部分平台上能取得显著优化

[^1]: 我队 2023 赛季雷达站部署于一台 Arch Linux 机子上。而本模块的前期开发是在一台 MacBook Air 上完成的。
