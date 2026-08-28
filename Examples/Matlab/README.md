# MT_IMU_MATLAB
### 程序功能

​	解析通过串口工具或MT上位机记录的美泰IMU的txt数据

### 程序目录

- decode_ZD_mode0.m
- decode_ZD_mode1.m
- decode_ZD_mode2.m
- decode_ZD_mode4.m
- decode_ZD_mode5.m
- double_checksum_func.m

### 使用方法

1. 确认当前IMU输出的数据模式，打开对应的解析m文件
1. 确认校验函数`double_checksum_func.m`处于当前工作目录下
2. 修改`filename = 'txt文件路径'`路径为用户文件的全局路径
3. 运行程序
4. 程序运行完成后，查看工作区的`data_out`文件，数组的各列含义详见代码注释

