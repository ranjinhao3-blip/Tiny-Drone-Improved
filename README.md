# Tiny-Drone-Improved

基于 [jonny-lekaiwu/Tiny-Drone](https://github.com/jonny-lekaiwu/Tiny-Drone) 复现与改进的 ESP32-S3 微型无人机项目。本人完成复刻、调试并在原项目基础上改进操纵、定点切换和诊断功能。原项目主要代码源于 [espressif/esp-drone](https://github.com/espressif/esp-drone)，保留原作者署名、[LICENSE](LICENSE) 与 [第三方声明](THIRD_PARTY_NOTICES.md)。

## 当前稳定版本

- 本次上传基于 `Tiny-Drone-v1.1.2.zip`，代码/固件为 **v1.1.2**，内部固件版本 `01.01.02`。
- 本人实飞反馈：**已调试，可以稳定飞行、定高、定点**（2026-11-17 补充）；尚未记录飞行时长、次数和误差统计。
- **已知不足：水平经过下方障碍物时会抬升，观察到抬升量约为障碍物高度。** 当前激光测距直接参与高度估计，不能将此版本描述为已实现跨障碍物保持相对起飞地面的固定高度；该问题尚未解决。
- 2026-11-17 本次整理只改文档，源码、参数和固件保持原 ZIP 字节；归档里的 `v1.1.3` 是此前补充实飞反馈的文档版本。

## 本人的主要改进

| 方向 | 工作内容 |
| --- | --- |
| 操纵响应 | 网页定时发包由 50 ms 调整到 20 ms，优化水平摇杆死区外的响应曲线。 |
| 松杆与定点 | 先零速制动、估计速度满足条件后捕获位置；完善切换位置模式时的外层 PID 状态处理。 |
| 网页交互 | 增加高低速切换、左右互斥的持续转向开关及断联/后台复位处理。 |
| 升降与降落 | 调整手动升降速度、近地下降和降推力缓停过程。 |
| 状态估计 | 修改停桨静止时的 Kalman 重力校正及相关观测有效性判断。 |
| 诊断与验证 | 补充传感器快照、串口诊断、CSV 记录、回归脚本和版本校验记录。 |

详见 [贡献说明与已知不足](docs/development-contributions.md)、[版本记录](CHANGELOG.md) 和 [实飞记录](docs/flight-verification-2026-11-04.md)。以上改进按已有增量记录整理；原硬件设计、图传、浏览器遥控和 OTA 等基础功能不作为本人原创。

## 归档说明

- 下方保留随 ZIP 提供的历史项目说明。历史分支、提交及标签信息用于追溯来源，本次从 ZIP 导入，不包含原本地 Git 历史，也不会自动生成这些标签。
- 气压/激光融合版本在另一条开发分支，本目录没有合入该试验方案。
- 本版配置 `ENABLE_DEVICE_ACTIVATION=0`，激活服务及对应电机锁定关闭；下方激活描述是原项目通用说明，当前行为以源码配置为准。
- 本版既有构建记录使用 ESP-IDF 5.5.4；下方原项目安装介绍仍保留 5.5.3。此次没有重新构建或实飞。

---

# Tiny-Drone 探秘者开源无人机

**简体中文** | [English](README_EN.md)

当前记录版本：**1.1.3（实飞确认，仅文档）**；代码和固件保持 **1.1.2（激光定高分支升降速度调整）** · [版本记录与每版修改说明](CHANGELOG.md)。**2026-11-04 用户确认：这一版本实机试飞比较稳定**，见[实飞反馈及固件校验记录](docs/flight-verification-2026-11-04.md)。本次不修改代码、参数或固件，不重新构建，内部固件版本仍为 `01.01.02`。

分支 `codex/laser-altitude-v1.1.2` 基于激光定高版本 `v1.1.1`：POSHOLD 网页低速档手动上升、下降均为 0.40 m/s，高速档均为 0.70 m/s，水平目标速度仍为 0.70/1.20 m/s。保存各传感器原始诊断和[本地监视器](docs/sensor-monitor.md)，监视器自动读取暂时关闭。沿用原有激光定高、Kalman 估速、松杆制动锁点和参考地面 20 cm 的 4 秒缓停。气压与激光融合版本 `v1.2.0` 保存在原分支 `codex/yaw-switch-v1.1.0`。操作说明见[POSHOLD 使用文档](docs/poshold-auto-takeoff.md)。

转向前先取消 YAW 锁定；约 2 秒转 90°，持续转动直到手动关闭。断联重连及页面进入后台后开关恢复关闭。见 [转向操作说明](docs/poshold-auto-takeoff.md)。

> 一款基于 ESP32-S3 开发的开源微型无人机，支持手机 App、手机浏览器和遥控器控制，并提供 Wi-Fi 图传与 RID 广播功能。

<p align="left">
  <img src="assets/TinyDrone/tiny-drone-overview.jpg" alt="Tiny-Drone 探秘者开源无人机" width="600">
</p>

## 项目简介

Tiny-Drone（探秘者开源无人机）是一款基于 ESP32-S3 开发的无人机玩具。项目主要代码源自 [ESP-Drone](https://github.com/espressif/esp-drone) 开源工程，并遵循  [GPL-3.0 License](LICENSE)，[硬件工程已开源](https://oshwhub.com/xiaochen_study/project_wsijdhkn)

主要功能：

- 支持 Android App 控制
- 支持手机浏览器控制
- 支持遥控器控制
- 支持 Wi-Fi 图传
- 支持 RID 广播
- 支持实名登记后通过 USB 激活，未激活时锁定电机（[使用说明](docs/activation.md)）
- 支持扩展气压计、激光测距及定点悬停模块

## 视频演示

[室内飞行](https://www.bilibili.com/video/BV1T7th6gEEx/)
[组装教程](https://www.bilibili.com/video/BV1CBtU6UEvK/) 

## 注意事项

> [!IMPORTANT]
> 固件根据芯片出厂 MAC 生成产品序列号。完成对应设备的实名登记后，使用配套网页通过 USB 激活；激活后才允许电机运行，RID 使用保存的登记号。详见[激活说明](docs/activation.md)。

- Wi-Fi 图传可能出现少量横纹，电池电压过低时会更加明显。
- 出现航向角偏移时，应调整电机高度和桨叶安装深度。
- 使用定高功能前，必须确保桨叶与电机已调整至合适角度，否则可能出现失控乱飞。
- 安装桨叶时，可用手抵住电机底部，避免用力过大冲破后盖并导致电机脱落。

## 硬件设计

### 电源

使用锂电池供电，经升压芯片升压后，再通过 LDO 为传感器和主控供电。

<p align="left">
  <img src="assets/TinyDrone/power-supply.png" alt="Tiny-Drone 电源电路" width="600">
</p>

### 状态指示灯及电源指示灯

<p align="left">
  <img src="assets/TinyDrone/status-and-power-led.png" alt="状态指示灯及电源指示灯电路" width="600">
</p>

### 传感器

使用 ZY-MPU6050 模块，便于手工焊接。

<p align="left">
  <img src="assets/TinyDrone/zy-mpu6050-sensor.png" alt="ZY-MPU6050 传感器电路" width="600">
</p>

### 主控及 ADC 电量采集

<p align="left">
  <img src="assets/TinyDrone/esp32s3-and-adc.png" alt="ESP32-S3 主控及 ADC 电量采集电路" width="600">
</p>

### 电机驱动

<p align="left">
  <img src="assets/TinyDrone/motor-driver.png" alt="电机驱动电路" width="600">
</p>

### 摄像头

<p align="left">
  <img src="assets/TinyDrone/camera-circuit.png" alt="摄像头电路" width="600">
</p>

### 扩展模块

支持 SPL06-001 气压计和 VL53L1X 定高模块，并预留定点悬停模块接口。

<p align="left">
  <img src="assets/TinyDrone/expansion-modules.png" alt="扩展模块接口电路" width="600">
</p>

## 软件与控制端
上电后手机扫描连接wifi:"TINY-DRONE_XXXXXXXXXXXX"，密码为"87654321"

- 无人机固件：[Tiny-Drone](https://github.com/jonny-lekaiwu/Tiny-Drone)
- Android App：[ESP-Drone-Android](https://github.com/jonny-lekaiwu/ESP-Drone-Android) 

- 扫码下载安卓app
  <p align="left">
    <img src="assets/android_app.png" alt="Android App">
  </p>

苹果手机可通过浏览器访问 `192.168.26.28` 进行控制。iOS App 正在规划中，将根据实际需求安排开发。

<p align="left">
  <img src="assets/TinyDrone/android-app-control.jpg" alt="Android App 控制界面" width="400">
</p>
<p align="left"><em>Android App 控制界面</em></p>

<p align="left">
  <img src="assets/TinyDrone/mobile-browser-control.jpg" alt="手机浏览器控制界面" width="400">
</p>
<p align="left"><em>手机浏览器控制界面</em></p>

## 实物展示

<p align="left">
  <img src="assets/TinyDrone/tiny-drone-photo-1.jpg" alt="Tiny-Drone 实物图一" width="600">
</p>

<p align="left">
  <img src="assets/TinyDrone/tiny-drone-photo-2.jpg" alt="Tiny-Drone 实物图二" width="600">
</p>

<p align="left">
  <img src="assets/TinyDrone/tiny-drone-photo-3.jpg" alt="Tiny-Drone 实物图三" width="600">
</p>

<p align="left">
  <img src="assets/TinyDrone/tiny-drone-photo-4.jpg" alt="Tiny-Drone 实物图四" width="600">
</p>

## 复刻无人机

复刻前可准备中温锡膏和加热台，以提高焊接效率。

### 硬件组装

<p align="left">
  <img src="assets/TinyDrone/hardware-assembly.png" alt="Tiny-Drone 硬件组装示意图" width="600">
</p>

1. **准备元器件**

   按照 BOM 清单购买对应元器件，也可以直接购买[物料包](https://item.taobao.com/item.htm?ft=t&id=1080586148318)。建议选择质量较好的元器件，以免影响飞行效果；ZY-MPU6050 和 OV2640 最好购买经过卖家测试的产品。

   <p align="left">
     <img src="assets/TinyDrone/components-and-materials.jpg" alt="Tiny-Drone 元器件与物料" width="600">
   </p>

2. **PCB 打板**

   在嘉立创打开项目工程并导出 PCB 下单。PCB 厚度请选择 **1.6 mm**，否则胶圈与 PCB 之间可能产生间隙，不利于固定电机位置。

3. **焊接元器件**

   - 使用 ZY-MPU6050 模块代替 MPU6050，以支持手工焊接陀螺仪传感器。
   - 焊接 FPC 座时，建议使用刀型 USB 电烙铁；出现连锡时，可使用助焊剂和松香拖开焊锡。
   - 焊接 TYPE-C 16Pin 接口时，可使用中温锡膏搭配小体积刀型电烙铁。

   - 贴片图

      <img src="assets/TinyDrone/SMT.png" width="360"/>

### 编译及烧录

* 基于esp-idf-v5.5.3编译
    - 下载windows版本 [esp-idf](https://dl.espressif.cn/dl/esp-idf/?idf=5.5.3)

  <img src="assets/download_idf.png" width="360"/> 

  ~~~
  ./build.bat tiny-drone
  idf.py flash monitor -p COMX
  ~~~

### 一键烧录
* 打开在线烧录[网站](https://espressif.github.io/esptool-js/)

* 打开连接识别到的COM并打开

  <img src="assets/connect.png" width="360"/> 

* 选择要烧录的文件，设置地址为0

  <img src="assets/choose_file.png" width="360"/> 

* 点击烧录，等待烧录完成

  <img src="assets/program.png" width="360"/> 

### 调整电机位置和桨叶安装深度

#### 电机位置

电机与底座之间应保留适当间隙，以便在发生碰撞时保护电机。

<p align="left">
  <img src="assets/TinyDrone/motor-position.jpg" alt="电机与底座的正确间隙" width="600">
</p>

#### 电机角度

电机应与水平面保持垂直，以避免航向角出现明显偏移。

<p align="left">
  <img src="assets/TinyDrone/motor-angle.jpg" alt="电机安装角度" width="600">
</p>

#### 桨叶深度

桨叶与电机之间应保留约 **2 mm** 的间距。

<p align="left">
  <img src="assets/TinyDrone/propeller-clearance.jpg" alt="桨叶与电机之间保留约 2 mm 间距" width="600">
</p>

## 故障排除

<details>
<summary><strong>打开电源后，拉油门没有反应</strong></summary>

观察状态指示灯是否处于慢闪状态。可能是 MPU6050 未通过自检，请同时确认无人机是否保持水平。

</details>

<details>
<summary><strong>撞到物体后，再次拉油门没有反应</strong></summary>

无人机可能检测到跌落并进入急停状态，请重启设备。

</details>

<details>
<summary><strong>摄像头没有反应</strong></summary>

检查日志，确认摄像头是否初始化成功。若初始化成功，请逐一检查信号线是否焊接良好，以及信号线是否与 GND、3.3 V 或其他信号线短路。

</details>

<details>
<summary><strong>起飞后，航向角总是偏向一侧</strong></summary>

如果无人机向右偏，可能是左侧电机位置过高。可将左侧电机调低，或将右侧电机适当调高。

</details>

<details>
<summary><strong>起飞后，其中一个桨叶不转</strong></summary>

电机可能因碰撞损坏，或内部接线已经断开，需要更换电机。如果更换电机后问题仍然存在，请为 MCU 相关信号线重新上锡，并检查是否存在虚焊。

</details>

<details>
<summary><strong>桨叶转动，但无人机无法起飞</strong></summary>

检查桨叶的型号、方向和安装位置是否正确。 

<p align="left">
  <img src="assets/TinyDrone/propeller-installation.png" alt="Tiny-Drone 桨叶正确安装方式" width="600">
</p>

</details>

## 贡献者

感谢TZ50109：改进无人机起飞、降落的稳定性。
