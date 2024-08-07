# KF-GINS-ECEF

[[中]](./readme_zh.md) &ensp; [[EN]](./readme.md)

## 数据集与项目概览
本项目在地心地固坐标系(ECEF系)下实现了GNSS与INS的松组合定位，内含初始对准算法、惯导机械编排算法、基于扩展卡尔曼滤波(EKF)的松组合算法，可实现鲁棒且高精度的定位。

本项目包含两个文件夹：
- `dataset`：数据集文件夹，包含武汉大学采集的三段动态实验数据。
- `GINS`：程序文件夹，包含解决方案文件和源码文件夹。源码文件夹进一步包含头文件、cpp文件和配置文件。

## 打开程序
使用Microsoft Visual Studio打开`GINS`文件夹中的解决方案文件，即可打开本程序。

## Eigen库配置
由于程序使用Eigen库管理矩阵，您需要先下载Eigen库。您可以从以下链接下载Eigen库：
[Eigen库下载](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## 项目配置步骤
1. **Eigen库路径配置**：在打开项目后，首先在属性内配置Eigen库的路径，以确保项目能够正常使用Eigen库。

2. **配置文件修改**：
   - 修改`main.cpp`内的配置文件路径。
   - 配置文件包括：
     - `friend.cfg`：友谊广场的配置文件。
     - `play_in.cfg`：操场内侧的配置文件。
     - `play_out.cfg`：操场外侧的配置文件。

3. **数据文件路径配置**：修改配置文件内指向数据文件的路径，包括以下五个路径：
   - IMU文件路径
   - GNSS文件路径
   - 参考结果文件路径
   - 处理结果文件路径
   - 对比结果文件路径

   这些文件都可以在`dataset`文件夹内找到，请根据您设备上的路径进行配置文件路径的修改。

## 程序运行
当所有路径问题都解决后，您可以在release模式下运行程序。运行后，您将得到处理结果文件和对比结果文件。利用这些结果文件，您可以绘制结果图并进行多种分析。

## 注意事项
- 确保所有路径配置正确，以避免程序运行时出现错误。
- 确保数据集文件的内容格式正确，格式需严格参考`dataset`内的数据集内容格式。
- 若遇到其他问题，可联系chengzhuogui@gmail.com。
