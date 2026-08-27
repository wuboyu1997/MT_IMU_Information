# MT_IMU_STM32H743
### 程序功能

​	通过uart2进行IMU配置和数据读取，解析读取到的数据并通过uart1打印

### 程序目录

- main.c:

  初始化外设

  发送IMU配置指令

  循环解析并打印IMU数据（受串口空闲中断触发）

- imu.c/imu.h:

  存储IMU配置指令、IMU数据结构

  IMU配置函数

  IMU数据解析函数

- check_alg.c/check_alg.h

  数据校验算法函数

- 其他：

  相关外设、启动、依赖文件
