# PointCloudDetection

RoboMaster 2023 哈尔滨工业大学（深圳） 南工骁鹰战队 雷达站点云定位模块

**注意：由于 2024 赛季代码尚未整理完毕，决定开源 2023 赛季源码的点云定位模块作为参考。**

> 为什么不开源其他模块？
> 其他模块非本人编写，且问题较多，故暂不开源。
> 点云定位模块是该方案最核心的模块，是本套方案最核心的创新点。可以直接替换双层神经网络方案的第一层机器人识别网络。

![演示效果](./doc/demo.png)

## 项目设计

点云定位系统的设计目标，是作为雷达站确定场上运动目标的第一层检测，为后续的目标识别提供位置信息，同时借助其获取的丰富三维信息，进行目标预测从而保证跟踪的连续性。

整体设计可参考 RMUC 2024 青年工程师大会录像 [BV1NE4m197pm](https://www.bilibili.com/video/BV1NE4m197pm) 以及 [幻灯片](./doc/一种基于点云聚类的雷达算法.pptx)

## 项目优势

相比上海交通大学在 2021 赛季开源的雷达站代码 [COMoER/LCR_sjtu](https://github.com/COMoER/LCR_sjtu) 相比，我们采用基于点云的目标检测与跟踪，具有视觉识别所不具有的高灵敏性以及不全可见目标的检测能力。与此同时，我们的处理效率更为高效，在正常赛场条件下可实现 500Hz 以上[^1]的处理速度，保证了雷达站的实时性。

## 项目架构

```
main.cpp                    主程序 *核心*
config.yaml                 配置文件
config.sample.yaml          样例配置文件
default.yaml                默认配置文件

Clustering.h/.cpp           聚类算法
Config.h                    配置读取
KalmanFilter.h/.cpp         卡尔曼滤波
PcReceiver.h/_**.cpp        点云接收
Recorder.h/.cpp             内录程序
TargetMap.h/.cpp            目标跟踪 *核心*
Transform.h/.cpp            坐标变换处理
VisualizerHelper.h/_**.cpp  可视化相关
VoxelGrid.h/.cpp            点云体素化 *核心*
```

## 协议与内录

### 激光雷达

本项目提供了第二代 Livox 激光雷达（HAP/Mid-360）的驱动。但实现不完善，只提供了 HAP 通信协议中约定的 32 位笛卡尔坐标系包的接收；且该驱动不提供发包指令的发送功能，建议搭配 [shirok1/pylivox2](https://github.com/shirok1/pylivox2) 使用。

**值得注意的是，本模块的内录格式是流式记录接收到的激光雷达的数据包，支持 zstd 压缩。**

### 定位结果

我们使用 MQTT 协议实现模块间通信，如果需要迁移至其他框架，在此进行修改即可。

定位结果数据结构（发送于主题 `pc_detected` ）：

```jsonc
{
  "enemies": [{
    "id": uint,                           // 目标编号
    "position": [float, float, float],    // 位置坐标（单位：mm）
    "velocity": [float, float, float],    // 速度矢量（单位：mm/s）
    "is_predict": bool,                   // 该数据是否为预测值
    "lost_time": uint,                    // 丢失跟踪的帧数
    "is_discarded": bool                  // 是否结束跟踪（目标被废弃）
  }]
}
```

该部分没有内录，有需要者可以用 MQTT Broker 进行录制。

## 部署教程

### 安装依赖

本项目是跨平台的[^2]，但为保证可复现性，我们以 Ubuntu 22.04 作为参考部署环境。由于本项目利用了源内版本的 Open3D 不具备的功能，必须手动编译安装 Open3D （见下一节）。

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

### 部署运行

运行程序前，必须保证运行目录（`$PWD`）下有以下文件：

- 配置文件
    + `config.yaml` 当前配置文件（可为空）
    + `default.yaml` 默认配置文件
- 资源文件（具体目录可由配置文件修改）
    + `resource/bg2align.stl` 配准参考场地模型
    + `resource/bg2filter.stl` 过滤参考场地模型

如果使用激光雷达内录，请一并准备好内录文件。

### 性能优化提示

- 使用较高的优化等级，本项目对浮点运算精度要求不高，可较为放心地启用优化
- 实际部署请使用 Release 配置
- 使用 Intel oneAPI ICX 编译器编译，在部分平台上能取得显著优化

[^1]: 在一台搭载 i7-12700H 处理器的笔记本上，开启 `-O3` 优化选项，使用 ICX 编译器编译，于读取 PCD 文件的模式下达成。实际运行受到激光雷达速度限制，能达到稳定 49Hz 的处理速率。
[^2]: 我队 2023 赛季雷达站部署于一台 Arch Linux 机子上。而本模块的前期开发是在一台 MacBook Air 上完成的。
