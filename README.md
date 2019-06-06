# ssl-homework

### feature

- 走固定路径点（VO任务的车控程序）

### 使用说明

1. 下载源码和protobuf.zip，解压到同一级目录下
2. 修改utils/param.h里的serialPort，改成对应发射机的端口
3. 使用MSVC2015 32bit编译

4. 连上发射机，使用Crazy连接上小车，确认连接成功后运行程序

### 其他

- 如果要修改路径点，打开algorithm/pathplanner.cpp，修改第18行的路径点
- 如果要修改小车运动参数，打开utils/param.h，修改第35行后的参数