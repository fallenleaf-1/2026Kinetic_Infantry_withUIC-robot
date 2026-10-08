# CLion 编译烧录使用方法（RoboMaster C 板 / STM32F407IG）

<center><b><font face="楷体">basic_framework + CMake + Ninja + arm-none-eabi-gcc + J-Link</font></b></center>

[TOC]

## 0. 结论速览

本工程用 **CMake + Ninja + arm-none-eabi-gcc** 编译，用 **J-Link** 烧录。完整流程就四步：

> **① CMake 编译 → ② 拿到 `.elf`/`.hex`/`.bin` 三件套（§3.4）→ ③ 选一种方式烧录（§4 / §5）→ ④ 复位运行确认（§5.1.4）**
> 日常在 CLion 里：「Run → New Embedded Configuration → Segger JLink」一键完成"编译 → 烧录 → 复位 → 挂调试"；只想烧录不调试时，用 J-Link Commander 一条命令。

| 我想做的事 | 用什么 | 见 |
| --- | --- | --- |
| 只编译 | CLion `Build`，或命令行 `cmake --build` | §3 |
| 搞懂 `.elf`/`.hex`/`.bin` 分别给谁用 | 构建产物详解 | §3.4 |
| 编译 + 烧录 + 调试（**推荐**） | Run/Debug 配置：`Embedded GDB Server` (Segger JLink) | §4 |
| 只烧录、不调试（日常） | J-Link Commander 脚本 | §5.1 |
| 图形界面烧录 / 救砖 / 量产 | J-Flash | §5.2 |
| 手动体会 CLion 调试底层原理 | `JLinkGDBServerCL.exe` + `arm-none-eabi-gdb` | §5.3 |
| 备份 / 回读芯片 Flash（防变砖） | J-Link Commander `savebin` | §5.1.2 |
| 看 RTT 日志 | `JLinkRTTClient.exe` | §6 |
| 看外设寄存器 | 调试会话的 `Peripherals` 视图 + `STM32F407.svd` | §4.3 |

> **一条重要结论（2026-10-08 在本项目实测）**
> 本机的 **OpenOCD 0.12.0 打不开 J-Link**：SEGGER 的内核驱动占用了探针的 USB 接口，OpenOCD 走 libusb 时会直接报
> `Failed to open device: LIBUSB_ERROR_NOT_FOUND` / `Error: No J-Link device found`。
> 所以 **Windows + J-Link 组合请走 SEGGER 官方工具链**（J-Link Commander / J-Link GDB Server / J-Flash / CLion 的 Embedded GDB Server）。
> OpenOCD 留给 DAP-Link、无线调试器等（见 §7.3），不要为了 J-Link 去用 Zadig 改 USB 驱动——那会让 J-Link 官方工具一起失效。

---

## 1. 环境清单（本机实测路径）

| 组件 | 版本 | 路径 / 说明 |
| --- | --- | --- |
| arm-none-eabi-gcc | 14.3.1 | `D:\STM32CUBECLT\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin`（**已在 PATH**） |
| arm-none-eabi-gdb / g++ / objcopy | 同上 | 与 gcc 同目录，工具链配置要用 |
| CMake | 4.3.1 | `D:\STM32CUBECLT\STM32CubeCLT_1.22.0\CMake\bin\cmake.exe`（已在 PATH） |
| Ninja | 1.13.2 | `D:\STM32CUBECLT\STM32CubeCLT_1.22.0\Ninja\bin\ninja.exe`（已在 PATH） |
| J-Link 软件 | V9.30（DLL V9.30） | `D:\JLink_V930\`，含 `JLink.exe`、`JLinkGDBServerCL.exe`、`JFlash.exe`、`JLinkRTTClient.exe`（**不在 PATH**） |
| OpenOCD | 0.12.0 | `D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe`（**仅供 DAP-Link**） |
| CLion | 2025.3.3 | `D:\clion\CLion 2025.3.3` |

一行行自检（PowerShell）：

```powershell
arm-none-eabi-gcc --version      # 期望：arm-none-eabi-gcc.exe (GNU Tools for STM32 14.3.rel1...) 14.3.1
cmake --version                  # 期望：cmake version 4.3.1
ninja --version                  # 期望：1.13.2
Test-Path D:\JLink_V930\JLinkGDBServerCL.exe   # 期望：True
```

**硬件**：J-Link 以 SWD 接 C 板（至少 **SWDIO / SWCLK / GND / VTref**），并且 **C 板必须上电**（电池或稳定电源；只靠 USB 给整机供电容易掉电导致识别失败）。

怎么快速判断"是没插好还是软件问题"——用 J-Link Commander 连一次看回显。下面是本机实测输出（为省篇幅删掉了寄存器转储等无关行，保留的都是原样）：

```
Connecting to J-Link via USB...O.K.
Firmware: J-Link V9 compiled Dec  8 2023 20:16:22
Hardware version: V9.70
S/N: 602719460
License(s): GDB, RDI, FlashBP, FlashDL, JFlash
VTref=3.330V

Connecting to target via SWD
Found SW-DP with ID 0x2BA01477
CPUID register: 0x410FC241. Implementer code: 0x41 (ARM)
Found Cortex-M4 r0p1, Little endian.
```

只要能看到 `Found Cortex-M4 r0p1` 和合理的 `VTref`（≈3.3 V），硬件链路就是通的，剩下都是软件/配置问题。
`VTref` 为 0 或报 `No J-Link device found`，先怀疑**板子没上电**或 SWD 线没接好。

---

## 2. CLion 工具链配置（只需做一次）

> **本工程的特殊性**：`CMakeLists.txt` 里自己写死了编译器
> `set(CMAKE_C_COMPILER arm-none-eabi-gcc)`，并强制 `set(CMAKE_BUILD_TYPE Debug)`。
> 因此 —— **arm-none-eabi 工具链必须在系统 PATH 上**（否则 CMake 直接报找不到编译器），并且 **不要** 给这个工程选 MinGW / MSVC 工具链。

1. `Settings` → `Build, Execution, Deployment` → `Toolchain`，点 `+` 新建一个 **System** 类型工具链：

   | 字段 | 值 |
   | --- | --- |
   | Name | `ARM (STM32CubeCLT)` |
   | C Compiler | `D:\STM32CUBECLT\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe` |
   | C++ Compiler | `...\arm-none-eabi-g++.exe` |
   | Debugger | `...\arm-none-eabi-gdb.exe` |

2. `Settings` → `Build, Execution, Deployment` → `CMake`，把 `Debug` profile 的 **Toolchain** 选成上面这个，然后 `Reload CMake Project`。

3. 本工程当前只有 `Debug` 一个 CMake profile，对应构建目录 **`cmake-build-debug/`**（CLion 自动创建）。你在 CLion 里编译/调试用的都是这个目录。

> 工具链如果指向了本机的 x86 gcc/gdb，会出现两类典型症状：链接阶段报架构不兼容；或者调试时 gdb 连不上 ARM 目标。出现这两类报错先回来检查这一节。

---

## 3. 编译（CMake 构建详解）

### 3.1 编译方法

- CMake 目标名就是 **`basic_framework.elf`**（`CMakeLists.txt` 里 `project(basic_framework C ASM)`）。
- CLion：直接 `Build` → `Build Project`（或 `Ctrl+F9`）。
- 产物：`CMakeLists.txt` 的 `POST_BUILD` 会自动在同一个构建目录里额外生成 **`.hex` 和 `.bin`**，也就是说你会拿到三个文件：

  ```
  cmake-build-debug/basic_framework.elf   # 调试用（带符号）
  cmake-build-debug/basic_framework.hex   # 烧录用（Intel HEX，地址自包含）
  cmake-build-debug/basic_framework.bin   # 纯二进制，需要显式指定烧录地址 0x08000000
  ```

### 3.2 命令行等价写法

```powershell
# 配置（只需一次）
cmake -B cmake-build-verify -S . -G Ninja
# 编译
cmake --build cmake-build-verify --target basic_framework.elf -j 24
```

> 这里用 `cmake-build-verify` 是**故意**的：**CLion 正在运行时**，别去动它的 `cmake-build-debug/`，否则可能和 CLion 的 CMake reload 抢文件。CLion 关掉后两个目录随便用。

> 另外注意：工程根目录可能存在一个 `build/` 目录，里面**只有 CMake 缓存而没有 `.elf`**——那是某次只 configure 没编译的残留。编译产物以 CLion 的 `cmake-build-debug/`（或你命令行 `-B` 指定的目录）为准，别在 `build/` 里找固件。

### 3.3 期望输出（2026-10-08 实测）

```
Memory region         Used Size  Region Size  %age Used
             RAM:       76768 B       128 KB     58.57%
          CCMRAM:           0 B        64 KB      0.00%
           FLASH:      103372 B         1 MB      9.86%
   text    data     bss     dec     hex  filename
 102016    1348   75424  178788   2ba64  basic_framework.elf
```

**链接期下面这类提示是正常的，不是错误**（newlib-nano + `-lnosys` 的标准产物）：

```
warning: _close is not implemented and will always fail
warning: _lseek is not implemented and will always fail
warning: _read / _write / _fstat / _isatty is not implemented and will always fail
```

注意 `CMakeLists.txt` 开了 `-Wall -Werror`：**任何告警都会直接中断编译**，先修干净再说。

### 3.4 构建产物详解 —— CMake 构建后怎么处理

构建成功后，构建目录（CLion 是 `cmake-build-debug/`，命令行示例是 `cmake-build-verify/`）里与"烧录/调试"相关的文件一览：

| 文件 | 是什么 | 给谁用 |
| --- | --- | --- |
| `basic_framework.elf` | 带**调试符号**（函数名/变量名/源码行号）与地址信息的完整映像 | 调试器（CLion / GDB / Ozone）；`gdb load`、Commander `loadfile` 也能直接烧它 |
| `basic_framework.hex` | Intel HEX 文本格式，**每行自带烧录地址**，由 `objcopy -Oihex` 生成 | J-Link Commander / J-Flash（**日常烧录首选**） |
| `basic_framework.bin` | 纯二进制裸数据，**不含任何地址信息**（从 Flash 最低地址 0x08000000 原样展开） | 必须显式补地址；常见于 DFU / SD 卡升级 / 上位机 OTA 等场景 |
| `basic_framework.map` | 链接器内存分配表：每个函数/变量在哪、占多大 | 分析体积、查栈/堆溢出（`-Wl,-Map` 自动生成） |

生成机制（`CMakeLists.txt` 的 `POST_BUILD` 钩子，编译成功后自动执行，不用手动跑）：

```
arm-none-eabi-objcopy -Oihex   basic_framework.elf basic_framework.hex
arm-none-eabi-objcopy -Obinary basic_framework.elf basic_framework.bin
arm-none-eabi-size  basic_framework.elf        # 控制台那段 text/data/bss 输出就是它
```

三个文件同源（都是同一个 `.elf` 转出来的），**选哪个只取决于烧录工具怎么吃地址**：

- `.hex` 自带地址 → "扔给谁都能烧"，日常烧录用它；
- `.bin` 裸数据 → 忘了补 `0x08000000` 轻则烧错位置，重则变砖。

**先确认产物是新的再烧**（改了代码却烧了旧固件是最常见的坑）：

```powershell
# 产物时间戳应晚于你最后一次改源码的时间
Get-Item cmake-build-debug\basic_framework.hex | Select-Object LastWriteTime
# 一条命令列出所有比某个源文件旧的构建产物（自行替换路径）
Get-ChildItem application,Src,modules,bsp -Recurse -Include *.c,*.h |
  Sort-Object LastWriteTime -Descending | Select-Object -First 3 FullName,LastWriteTime
```

### 3.5 构建之后的下一步（流程决策）

```
编译成功，拿到 .elf / .hex / .bin / .map
        │
        ├─ 要打断点 / 单步 / 看变量 ──→ §4   CLion Embedded GDB Server（吃 .elf）
        ├─ 只想烧进去跑（日常）     ──→ §5.1 J-Link Commander 脚本（吃 .hex）★
        │                              或 §5.2 J-Flash（图形界面 / 救砖 / 量产）
        └─ 要看运行日志             ──→ §6   RTT（附着在任一已存在的 J-Link 连接上）
```

烧录永远是「**编译 → 选产物 → 烧写 → 复位运行**」四步。§4/§5 的几种工具只是在重复这四步，区别只是形态：CLion 图形化一键、Commander 脚本、还是 GDB 协议。

---

## 4. 烧录 + 调试（推荐：Embedded GDB Server + Segger J-Link）

这条路径用的是 SEGGER 自己的 `JLinkGDBServerCL.exe`，走 SEGGER 驱动，**稳定且不需要额外配置**。

### 4.1 新建配置

主菜单 `Run` → **`New Embedded Configuration`**（或在 `Run` → `Edit Configurations` 里 `+` → `Embedded GDB Server`）。

> **⚠️ 界面版本差异（2026-10-08 在 CLion 2025.3.3 实测）**
> 新版 CLion 的 Embedded GDB Server 是**通用单页表单**：没有老教程向导里的"GDB Server type: Segger JLink"下拉，也没有第二页的 Interface / Speed / Device——SWD、速率、芯片型号**全部通过 GDB Server args 传给 `JLinkGDBServerCL.exe`**。按下面这张表填：

| 界面字段 | 填什么 |
| --- | --- |
| Name | `JLink Debug`（随意） |
| Target | `basic_framework.elf`（CMake target） |
| Executable binary | `cmake-build-debug\basic_framework.elf`（选了 Target 通常自动带出；没带就点 `...` 手动选文件） |
| Debugger | `Bundled gdb multiarch` 可直接用；或换成 §2 的 `arm-none-eabi-gdb.exe` |
| Upload executable | `Always`（每次都下载；`If updated` 也可，按产物新旧决定） |
| 'target remote' args | `tcp:localhost:2331` |
| **GDB Server** | `D:\JLink_V930\JLinkGDBServerCL.exe`（**留空会报 `GDB Server executable is not specified`**） |
| **GDB Server args** | `-device STM32F407IG -if SWD -speed 4000 -port 2331`（老向导第二页的 SWD / 4000 kHz / STM32F407IG 三件套就是它） |
| Advanced GDB Server Options | 展开：**Reset command** = `monitor reset`；**Startup delay** 默认即可，偶发 "cannot connect" 给 `500 ~ 1000` ms |

> `-port 2331` 必须和 'target remote' args 里的端口一致。

### 4.2 启动

点 **Debug**（绿色小虫子）即完成：编译 → 拉起 `JLinkGDBServerCL` → 下载 `.elf` → 复位 → 挂上调试器。

> 只烧录不调试时，别用这个配置"Run"（GDB Server 配置本质是调试会话）。请走 §5，更快也更干净。

### 4.3 调试时能用到的东西

- **Peripherals 视图**：在配置里把 **SVD 文件** 指到仓库根目录自带的 `STM32F407.svd`，就能像 Ozone 一样看寄存器。
- **RTOS 视图**：本工程跑 FreeRTOS，可以按任务维度看栈和状态。
- **MCU 复位按钮**（调试器左侧栏）：发 §4.2 里配置的 reset command。
- 内存 / 反汇编视图、变量实时监视（Live Watch）。

### 4.4 如果探针是"克隆版"

SEGGER 官方新版（本机 V9.30）会拒绝克隆探针，报 `The JLink is defective` / `clone version` 之类。
- 本机实测是 **官方授权探针**：`License(s): GDB, RDI, FlashBP, FlashDL, JFlash`，不存在此问题。
- 若你换到克隆探针上遇到该报错：`.Doc/VSCode+Ozone使用方法.md` 中"替换 `JLinkARM.dll`"一节就是历史上绕过该检查的做法；更干净的办法是 **改用 DAP-Link + OpenOCD**（见 §7.3）。

---

## 5. 只烧录不调试 —— J-Link 软件使用详解

### 5.0 J-Link 软件家族速览

`D:\JLink_V930\` 里与日常开发相关的可执行文件：

| 程序 | 形态 | 干什么 | 详见 |
| --- | --- | --- | --- |
| `JLink.exe` | 命令行（交互 / 脚本） | 烧录、复位启停、读写内存、擦除、备份 Flash | §5.1 |
| `JFlash.exe` | 图形界面 | 生产级烧录（擦除→编程→校验→启动一键完成）；救砖、量产 | §5.2 |
| `JLinkGDBServerCL.exe` | 后台服务 | 把 J-Link 桥接成 GDB 调试通道；CLion「Embedded GDB Server」底层就是它 | §4 / §5.3 |
| `JLinkRTTClient.exe` / `JLinkRTTViewer.exe` | 命令行 / GUI | 查看 RTT 日志 | §6 |

**怎么选**：日常烧固件用 §5.1 脚本（一条命令、可绑 CLion 快捷键）；想"亲眼看到"擦写过程、或板子变砖要救，用 §5.2 J-Flash；打断点调试走 §4；想搞懂 CLion 调试背后发生了什么，看 §5.3。

### 5.1 方式 A（日常推荐）：J-Link Commander

#### 5.1.1 先认识：交互模式

PowerShell 里直接运行（不带脚本参数），进入 `J-Link>` 提示符：

```powershell
D:\JLink_V930\JLink.exe -device STM32F407IG -if SWD -speed 4000
```

| 参数 | 含义 |
| --- | --- |
| `-device STM32F407IG` | 芯片在 SEGGER 设备库中的名字（C 板 F407IGHx 用它即可） |
| `-if SWD` | 两线 SWD 接口（J-Link 也支持 JTAG，C 板走 SWD） |
| `-speed 4000` | 4 MHz；连线过长或不稳时降到 `1000` |

**连接在第一条需要目标的命令时才建立**。关键回显（本机实测，寄存器转储略）：

```text
Connecting to J-Link via USB...O.K.
Firmware: J-Link V9 compiled Dec  8 2023 20:16:22
Hardware version: V9.70
S/N: 602719460
License(s): GDB, RDI, FlashBP, FlashDL, JFlash
VTref=3.330V            ← 探针实测目标板电压：≈3.3V=已上电；0V=先查供电！
Found SW-DP with ID 0x2BA01477
Found Cortex-M4 r0p1, Little endian.    ← 芯片识别成功，SWD 链路通
```

#### 5.1.2 常用命令参考（在 `J-Link>` 提示符下，全小写）

| 命令 | 作用 | 说明 |
| --- | --- | --- |
| `h` | 暂停 CPU（halt） | |
| `r` | 复位 | 复位后 CPU 处于 halt，**还要 `g` 才会跑** |
| `g` | 运行（go） | |
| `loadfile <文件> [地址]` | 烧录 `.hex` / `.bin` / `.mot` | `.hex` 自带地址不用补；**`.bin` 必须补地址**：`loadfile app.bin 0x08000000` |
| `erase` | 整片擦除 Flash | 救砖用，平时不必 |
| `mem32 <addr>, <个数>` | 按 32 位读内存 | 例：`mem32 0x08000000, 4` 读向量表头 |
| `w4 <addr>, <值>` | 写一个 32 位字 | |
| `savebin <文件>, <addr>, <字节数>` | 把目标内存备份成 bin | 例：`savebin backup.bin 0x08000000 0x100000` 备份整片 1MB Flash |
| `si SWD` / `speed 4000` / `device STM32F407IG` | 会话中途改接口 / 速率 / 芯片 | 脚本模式常用这三件套 |
| `qc` | 退出并断开 | 断开后目标继续运行 |

上手建议先试两条：

- `mem32 0x08000000, 4` —— 能读出数据就说明链路通；
- `savebin backup.bin 0x08000000 0x100000` —— 给板子做全片备份，**刷坏了能原样救回来**。

向量表自检技巧：`mem32 0x08000000, 4` 的输出里，第 1 个字是**初始栈顶指针**（应落在 `0x2000xxxx`~`0x2002xxxx` 的 RAM 区），第 2 个字是 **Reset_Handler 入口**（应为 `0x0800xxxx` 且最低位是 1，Thumb 位）。两个值都合理，说明 Flash 里确实是一份能跑的固件。

#### 5.1.3 脚本模式（日常烧录就用这个）

在**工程根目录**新建脚本文件 **`flash.jlink`**（文件名建议就用这个，下面 §5.1.3 末尾的外部工具参数可以直接照抄；脚本放哪里都行，但路径要和调用时一致）：

```text
si SWD
speed 4000
device STM32F407IG
r
h
loadfile "<工程绝对路径>\cmake-build-debug\basic_framework.hex"
r
g
qc
```

逐行含义：`si/speed/device` 配置链路 → `r`+`h` 复位并暂停 → `loadfile` 擦写校验 → `r`+`g` 复位运行 → `qc` 断开退出。

执行：

```powershell
D:\JLink_V930\JLink.exe -CommanderScript flash.jlink -ExitOnError 1 -NoGui 1
```

> 脚本里的 `<工程绝对路径>` 换成实际路径，例如 `D:\traeproject\2026Kinetic_Infantry_withUIC\robot`；
> 如果你用的是 CLion 默认 profile，就把 `cmake-build-debug` 写进去（本文档 §3.2 命令行示例用的 `cmake-build-verify`，两者只要和实际产物目录一致即可）。

2026-10-08 实测关键输出（说明"确实写进 Flash 且校验通过"）：

```
Found SW-DP with ID 0x2BA01477
Found Cortex-M4 r0p1, Little endian.
Downloading file [D:\...\basic_framework.hex]...
Comparing flash   [100%] Done.
Erasing flash     [100%] Done.
Programming flash [100%] Done.
J-Link: Flash download: Bank 0 @ 0x08000000: 2 ranges affected (131072 bytes)
J-Link: Flash download: Program & Verify speed: 290 KB/s
O.K.
```

要点：

- `loadfile` 的行为是 **Compare → Erase → Program & Verify**：**先比对，内容一致就直接 `Skipped. Contents already match`（什么都不写）；不一致才擦写**，写完**自动校验**。所以它天然幂等，重复执行无害——同版本固件重烧会看到 Skip 而不是报错。
- 结尾的 `r`（复位）+ `g`（运行）很关键；`qc` 断开 J-Link 后目标继续跑。少了 `g`，板子会停在复位态"像没烧一样"。
- 只要看到 `Program & Verify ... O.K.`（或 `Contents already match`）就算成功。

**做成 CLion 一键**：`Settings` → `Tools` → `External Tools` → `+`

| 字段 | 值 |
| --- | --- |
| Name | `Flash C Board (J-Link)` |
| Program | `D:\JLink_V930\JLink.exe` |
| Arguments | `-CommanderScript "$ProjectFileDir$\flash.jlink" -ExitOnError 1 -NoGui 1` |
| Working directory | `$ProjectFileDir$` |

之后 `Tools` → `External Tools` → `Flash C Board (J-Link)` 即可，还可以在 `Settings` → `Keymap` 里给它绑个快捷键。

#### 5.1.4 烧完怎么确认"真的在跑"

- `qc` 退出后目标已自动运行（脚本里 `r`+`g` 的作用）；板载现象（电机使能声、灯效）是最直接的证据。
- 接 §6 的 RTT 客户端看日志输出。
- 不放心就再进一次 Commander：`mem32 0x08000000, 4` 抽查向量表（自检方法见 §5.1.2）。

### 5.2 方式 B：J-Flash（图形界面 / 量产 / 救砖）

仓库根目录已经有 `stm32.jflash`（`ST STM32F407IG`、Bank 0 @ `0x08000000`、自动擦除/编程/校验）。图形界面操作流程：

1. 打开 `D:\JLink_V930\JFlash.exe`；
2. `File → Open Project`，选仓库根目录的 `stm32.jflash`；
3. `Target → Connect`，下方 Log 页出现 `Cortex-M4 identified` 即链路通；
4. `File → Open data file`，选 `cmake-build-debug\basic_framework.hex`；
5. `Target → Production Programming`（即工具栏 **Auto** 按钮）：擦除 → 编程 → 校验一气呵成，Log 页逐行回显进度；
6. `Target → Start Application` 启动程序。

典型使用场景：

- 想**直观看到**擦/写/校验的进度条和字节范围；
- **救砖**：`Target → Manual Programming → Erase chip` 先整片擦掉再重烧（等价 Commander 的 `erase`）；
- 量产 / 给多块板子反复烧同一固件。

命令行等价（无 GUI，脚本 / CI 可用），`Makefile` 的 `download_jlink` 目标用的就是它：

```powershell
D:\JLink_V930\JFlash.exe -openprj"D:\traeproject\2026Kinetic_Infantry_withUIC\robot\stm32.jflash" -open"D:\traeproject\2026Kinetic_Infantry_withUIC\robot\cmake-build-debug\basic_framework.hex",0x8000000 -auto -startapp -exit
```

2026-10-08 实测该命令退出码 `0`。J-Flash 有自己的窗口和 Log 页，细节以它为准；`-auto` 表示按工程里的生产设置（擦除+编程+校验）一路跑完，`-startapp` 表示烧完启动程序。

> `JFlash.exe` 不在 PATH，必须写全路径。若把 `D:\JLink_V930` 加进 PATH，那么 `Makefile` 里的 `mingw32-make download_jlink` 也能直接用。

### 5.3 进阶：手动跑 GDB Server —— 看懂 CLion 调试背后发生了什么

CLion 的 `Embedded GDB Server` 配置（§4）本质就是自动执行下面两步。手动做一遍，出问题时你就知道该盯哪个进程：

**第 1 步：启动 GDB Server**（它占住 J-Link，在 2331 端口等 GDB 接入）：

```powershell
D:\JLink_V930\JLinkGDBServerCL.exe -device STM32F407IG -if SWD -speed 4000 -port 2331
```

看到 `Waiting for GDB connection...` 即就绪。**真正执行烧写/调试苦力活的就是这个窗口**——GDB 侧报错时，先看它的日志。

**第 2 步：另开一个终端，用 ARM GDB 连上去下载并调试：**

```powershell
arm-none-eabi-gdb cmake-build-debug\basic_framework.elf
```

```text
(gdb) target remote localhost:2331   # 连上 GDB Server
(gdb) monitor reset                  # 复位并 halt（走 J-Link 复位策略）
(gdb) load                           # 把 .elf 擦写进 Flash 并校验
(gdb) b main                         # 在 main 设断点
(gdb) c                              # 放行运行，命中断点后进入交互调试
```

排错要点：

- GDB Server 的 `-port` 与 GDB 的 `target remote localhost:<端口>` 必须一致；
- **J-Link 同一时刻只能服务一个进程**：GDB Server 开着时别再开 Commander / J-Flash；
- 结束调试：GDB 里 `kill` + `quit`，GDB Server 窗口 `Ctrl+C` 退出——否则 J-Link 一直被占着，下次烧录报"设备被占用"。

### 5.4 ⚠️ 不要用错那个配置（亲测踩过的坑）

工程的 `.idea` 里预先存在一个名为 **`basic_framework.elf`** 的 **`CMake 应用程序`（CMake Application）** 运行配置 —— 这是 CLion 给可执行 target 自动生成的**默认模板，和烧录毫无关系**。

它的 **`Executable` 字段默认是构建产物 `basic_framework.elf`**。如果把 `.cfg` 之类的路径填进这个字段，点 Run 就会得到：

```
运行 'basic_framework.elf' 时出错
Cannot run program "D:\traeproject\2026Kinetic_Infantry_withUIC\robot\openocd_jlink.cfg"
(in directory "D:\traeproject\2026Kinetic_Infantry_withUIC\robot"):
CreateProcess error=193, %1 不是有效的 Win32 应用程序
```

原因很直白：**CLion 真的把这个 `.cfg` 当成可执行程序去启动了**（`CreateProcess`），而 `.cfg` 只是 OpenOCD 的文本配置文件，永远不可能执行。CLion 会把这个错误的值记在 `.idea/workspace.xml` 里，形如：

```xml
<configuration name="basic_framework.elf" type="CMakeRunConfiguration" ... RUN_PATH="$PROJECT_DIR$/openocd_jlink.cfg">
```

**怎么修（二选一）**：

- **推荐：直接删掉这个配置。** `Run` → `Edit Configurations` → 选中 `basic_framework.elf`（类型显示为 `CMake 应用程序`）→ 点 `−` 删除。ARM 固件不可能在 Windows 上跑，这个配置没有任何存在价值。
- 或者把它的 **`Executable`** 下拉改回 `basic_framework.elf`（构建目标），而不是自定义路径。

> **烧录不要走这个配置。** 烧录请用 §4 的 `Embedded GDB Server`（推荐），或 §5.1 / §5.2 的纯烧录方式。
> 另外注意方向：**用 J-Link 时本来就不该走 OpenOCD**（见 §0 的驱动原因），所以 `openocd_*.cfg` 不应该出现在任何"可执行文件 / 程序 / GDB Server"字段里 —— 它只属于 OpenOCD 配置里的 **Board config file** 字段，且只在用 DAP-Link 时才有意义（§7.3）。

---

## 6. 看日志（RTT）

```powershell
D:\JLink_V930\JLinkRTTClient.exe
```

- 前提：目标板在运行，且已经有一个 J-Link 连接存在（先跑 §4 的调试会话，或先执行一次 §5.1）。
- **两个 RTT 工具的区别**：`JLinkRTTClient.exe` **附着到已存在的连接**（GDB Server / Commander 会话），可以边调试边看；`JLinkRTTViewer.exe` 是独立 GUI，会**自己占用 J-Link**——调试会话开着时它连不上（同一 J-Link 同一时刻只能被一个进程使用）。
- **一个容易被坑的差异**：`Makefile` 里定义了 `-DDISABLE_LOG_SYSTEM`，而 **`CMakeLists.txt` 里没有**。所以 **用 CLion / CMake 编出来的固件，`LOGINFO` / `LOGWARNING` / `LOGERROR` 是开启的**（定义见 `bsp/log/bsp_log.h`），日志会走 RTT 通道。
- 如果嫌日志占资源，可以自己在 `CMakeLists.txt` 的 `add_definitions()` 里加上 `-DDISABLE_LOG_SYSTEM`。关掉日志后，初始化期因配置错误停机的位置（那些 `LOGERROR + while(1)`）改用 `BSP_FaultCode` 变量 + PC / 调用栈来定位。

---

## 7. 故障排查

### 7.1 症状速查表

| 症状 | 原因 | 处置 |
| --- | --- | --- |
| `arm-none-eabi-gcc: command not found` / CMake 报找不到编译器 | 工具链不在 PATH | 把 `D:\STM32CUBECLT\...\GNU-tools-for-STM32\bin` 加入 PATH，或 §2 里把 CLion 工具链指过去 |
| CLion 提示工具链不匹配、或链接报架构错误 | 选了 MinGW / MSVC 工具链 | 见 §2，改用 System + ARM 工具链 |
| `Error: No J-Link device found`、`LIBUSB_ERROR_NOT_FOUND` | 用 OpenOCD 连 J-Link，USB 接口被 SEGGER 驱动占用 | **改用 §4 / §5 的 SEGGER 工具**；OpenOCD 留给 DAP-Link |
| 点 Run 报 `Cannot run program "...openocd_jlink.cfg" ... CreateProcess error=193, %1 不是有效的 Win32 应用程序` | 名为 `basic_framework.elf` 的 **`CMake 应用程序`** 配置的 **`Executable`** 被填成了 `.cfg` 文件，CLion 真的去执行它 | 见 §5.4：删掉该配置，或把 `Executable` 改回 `basic_framework.elf` 构建目标 |
| `Cannot connect to target`、`target not halted` | 板子没上电 / SWD 接线 / 速度过高 | 确认板子上电（`VTref` 应≈3.3 V）、检查 SWDIO/SWCLK/GND、把速度降到 `1000` kHz 再试 |
| 烧录显示 O.K. 但板子没反应 | CPU 被留在 halt（脚本少了 `g`） | 用 §5.1.3 的 `r` + `g`，或 OpenOCD 用 `reset run` 而不是 `reset` |
| 改了代码但板子行为没变 | 烧的是旧产物（构建目录不对 / 没重新编译） | 见 §3.4：核对产物时间戳，确认烧的就是刚构建的那份 `.hex` |
| 编译中断在某条告警上 | `-Wall -Werror` | 修掉告警 |
| 中文注释乱码 | 文件编码不对 | 全工程统一 **UTF-8**（见 `.Doc/架构介绍与开发指南.md`） |
| 探头报"被占用"/连不上 | 同时开了多个会话 | J-Link 同一时刻只能被一个进程用：关掉 CLion 调试会话、RTT Client/Viewer、J-Flash、手动 GDB Server 再烧 |
| 命令行编译时 CLion 行为异常 | 两个进程抢同一个构建目录 | CLion 开着时命令行用 `cmake-build-verify`，别用 `cmake-build-debug` |

### 7.2 一页速查

```powershell
# 编译
cmake -B cmake-build-verify -S . -G Ninja
cmake --build cmake-build-verify --target basic_framework.elf -j 24

# 烧录（J-Link Commander，推荐）
D:\JLink_V930\JLink.exe -CommanderScript flash.jlink -ExitOnError 1 -NoGui 1

# 烧录（J-Flash，仓库自带工程）
D:\JLink_V930\JFlash.exe -openprj"stm32.jflash" -open"cmake-build-debug\basic_framework.hex",0x8000000 -auto -startapp -exit

# 备份整片 Flash：进交互模式执行
#   D:\JLink_V930\JLink.exe -device STM32F407IG -if SWD -speed 4000
#   J-Link> savebin backup.bin 0x08000000 0x100000

# 看日志
D:\JLink_V930\JLinkRTTClient.exe
```

### 7.3 什么时候才用 OpenOCD

**DAP-Link / 无线调试器**（不是 J-Link）时，用 OpenOCD + 仓库自带的 `openocd_dap.cfg`：

```powershell
openocd -f openocd_dap.cfg -c "init; halt" -c "flash write_image erase cmake-build-debug/basic_framework.hex" -c "reset run; shutdown"
```

这就是 `Makefile` 里 `download_dap` 目标的等价写法。注意两点：

- `.hex` 自带地址，**不要**再传 `0x08000000`；只有烧 `.bin` 时才需要 `... basic_framework.bin 0x08000000`。
- 结尾用 **`reset run`**（复位并运行）。如果用 `reset`，OpenOCD 默认会把 CPU 停在 halt，板子看着就像"烧完不动"。
- 在 CLion 里对应的图形化配置是 **`OpenOCD Download & Run`**（`Run` → `Edit Configurations` → `+`）：`Run` 按钮 = 下载并复位，`Debug` 按钮 = 下载 + 复位 + 挂调试；板级配置文件填仓库根的 `openocd_dap.cfg`（也可以点 `Copy to Project & Use` 复制一份再改）。用 J-Link 时这个配置会因为 §0 说的驱动原因连不上。

---

## 8. 2026-10-08 本机实测记录

| 项 | 结果 |
| --- | --- |
| 编译 | `cmake --build cmake-build-verify --target basic_framework.elf` 成功，FLASH 103372 B (9.86%)、RAM 76768 B (58.57%) |
| 探针 | J-Link V9.70 硬件 / DLL V9.30，S/N `602719460`，`License(s): GDB, RDI, FlashBP, FlashDL, JFlash`（官方授权） |
| 目标 | `Found SW-DP with ID 0x2BA01477`、`Found Cortex-M4 r0p1, Little endian`、`VTref=3.330V` |
| 烧录 | `loadfile` → Compare / Erase / Program & **Verify** 全部 `O.K.`，`Bank 0 @ 0x08000000: 2 ranges affected (131072 bytes)` |
| 烧录幂等性 | 同版本固件重复 `loadfile`：`Skipped. Contents already match`，不擦不写、无副作用（同日再烧实测） |
| OpenOCD | ❌ `Failed to open device: LIBUSB_ERROR_NOT_FOUND` / `No J-Link device found`（SEGGER 驱动占用 USB，DAP 与 JLink 两种 cfg 均失败） |
| J-Flash | ✅ 命令行退出码 `0` |

---

## 附：相关文件在工程中的位置

| 文件 | 作用 |
| --- | --- |
| [`CMakeLists.txt`](../CMakeLists.txt) | 构建脚本：编译器、`-Werror`、链接脚本、`POST_BUILD` 生成 `.hex/.bin` |
| [`STM32F407IGHx_FLASH.ld`](../STM32F407IGHx_FLASH.ld) | 链接脚本（C 板 STM32F407IGHx） |
| [`STM32F407.svd`](../STM32F407.svd) | 调试用外设寄存器描述（Peripherals 视图） |
| [`stm32.jflash`](../stm32.jflash) | J-Flash 烧录工程（Bank 0 @ 0x08000000） |
| [`openocd_dap.cfg`](../openocd_dap.cfg) / [`openocd_jlink.cfg`](../openocd_jlink.cfg) | OpenOCD 配置（DAP-Link / J-Link） |
| [`bsp/log/bsp_log.h`](../bsp/log/bsp_log.h) | 日志宏与 `DISABLE_LOG_SYSTEM` 开关 |
| [`.Doc/VSCode+Ozone使用方法.md`](./VSCode+Ozone使用方法.md) | 工具链原理、GDB/OpenOCD 关系、Ozone 可视化调试 |
