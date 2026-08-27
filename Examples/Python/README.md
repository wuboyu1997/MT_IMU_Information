# MT_IMU_Python.py
### 程序功能

​	配置IMU，读取IMU数据并解析，将解析到的结果实时打印到窗口中

### 快速开始

1. 安装依赖：

   `pip install pyserial`

2. 使用产品配套线束连接产品和PC

3. 快速运行：

   `python MT_IMU_Python.py COM3`

   把COM3换成实际使用的串口(如Windows: `COM3`, `COM4`；Linux: `/dev/ttyUSB0`)

   可选参数列表：

   |   `--baud `   | 通信波特率 | 默认 `460800`(选择产品目前的波特率)                          |
   | :-----------: | ---------- | ------------------------------------------------------------ |
   |   `--rate`    | 数据刷新率 | `0=1Hz, 1=2Hz, 2=5Hz, 3=10Hz, 4=20Hz, 5=25Hz, 6=50Hz, 7=100Hz(默认), 8=200Hz, 9=400Hz` |
   |   `--mode`    | 输出模式   | `0=Mode0(默认), 1=Mode1, 2=Mode2, 3=Mode3, 4=Mode4, 5=Mode5` |
   |   `--sync`    | 时间同步   | `0=OUT(默认), 1=IN, 2=RMC`                                   |
   | `--no-config` | 跳过配置   | 如果 IMU 已经配置好，加这个参数跳过配置                      |

​		例如：

- `		python MT_IMU_Python.py COM3 --baud 460800 --rate 7 --mode 0 --sync 0`

  表示使用460800的波特率进行数据读写，将IMU配置为：刷新率100Hz & mode0输出模式 & sync_out时间同步模式。

- `		python MT_IMU_Python.py COM3 --no-config`

​		表示直接使用IMU内部配置进行数据读取、打印
