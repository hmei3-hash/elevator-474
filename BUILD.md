# 用 PlatformIO 编译

## 为什么换掉 Arduino IDE

Arduino IDE **只编译 sketch 文件夹里的文件**。`logic/pid.c` 在 `ElevatorA/` 外面，所以头文件能找到、实现从来没被编译——`control.cpp` 编译通过，链接时报 `undefined reference to pid_init`，指着一个明明存在的函数。

PlatformIO 的 `build_src_filter` 直接解决这个：`logic/` 作为目标构建的一部分参与编译，**和 host 测试跑的是同一份源码，没有副本会漂移**。

## 一次性设置

```bash
pip install platformio        # 或者装 VS Code 的 PlatformIO IDE 扩展
```

## 文件改动

| 原来 | 现在 | 原因 |
|---|---|---|
| `ElevatorA/ElevatorA.ino` | `ElevatorA/ElevatorA_main.cpp` | PlatformIO 直接编译 `.cpp` |
| `ElevatorB/ElevatorB.ino` | `ElevatorB/ElevatorB_main.cpp` | 同上 |
| — | `platformio.ini` | 两个 environment |
| `logic/pid.h`、`filter.h` | 加了 `extern "C"` 包装 | 见下 |

**把两个 `.ino` 删掉。** 留着的话 PlatformIO 可能同时把它们也编进去，得到重复定义。

### `.ino` 改 `.cpp` 有一个隐含前提

Arduino IDE 会给 `.ino` 自动生成前向声明，`.cpp` 不会。所以**每个函数必须在使用前定义**。当前两个文件已经满足（所有任务函数都是 `static` 且定义在 `setup()` 之前），改名不需要动代码。以后往里加函数时要注意这一点。

### `extern "C"` 是必须的

`logic/*.c` 按 C 编译，`control.cpp` 按 C++ 编译。没有包装的话，调用方发出的是 C++ 修饰过的符号名，定义方是纯 C 符号名，**链接时对不上**——报错还是 `undefined reference`，但原因和上面那个完全不同。

包装写在 `pid.h` 和 `filter.h` 里，`#ifdef __cplusplus` 保护，所以 host 测试（纯 C）不受影响。

## 编译和烧录

```bash
pio run -e boardA                 # 编译 A 板
pio run -e boardA -t upload       # 编译并烧录
pio run -e boardB -t upload       # B 板
pio device monitor -e boardA      # 串口监视器，115200
```

两块板同时插着的时候要指定端口：

```bash
pio run -e boardA -t upload --upload-port COM4
```

## 板级设置已经写进 platformio.ini

| 设置 | 值 | 为什么 |
|---|---|---|
| `board` | `esp32-s3-devkitc-1` | |
| `memory_type` | `qio_opi` | **N16R8 是八线 PSRAM**，设错了板子会启动异常且没有有用的报错 |
| `ARDUINO_USB_CDC_ON_BOOT` | `1` | 不设的话串口一个字都不出 |
| `-Wall -Wextra` | | V&V 的 A-V1 要求"no new warnings" |

## host 测试不受影响

```bash
cd logic/test
gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c ../filter.c -lm
./test_logic
```

跑的还是 `logic/` 里那唯一一份源码——**这正是 host 测试能作为固件证据、而不只是某个相似副本的证据的原因**。

## 已知警告

`LiquidCrystal I2C claims to run on avr architecture` —— 只是元数据没声明 esp32，库本身在 ESP32 上工作正常。在测试日志里注明即可，不影响功能。
