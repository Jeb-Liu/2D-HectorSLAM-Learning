# 2D-HectorSLAM-Learning
## 利用搭载了激光雷达的小车实际采集的数据，复原环境，并绘制路径
输入：每个时刻的激光雷达水平数据
输出：小车路径，点云地图

## 数据来源及格式
数据来自2D Cartographer Backpack – Deutsches Museum（公开）
https://google-cartographer-ros.readthedocs.io/en/latest/data.html#d-cartographer-backpack-deutsches-museum

完整的数据包含了激光雷达的水平方向与垂直方向数据，以及加速度计数据
此例中仅使用了水平方向的激光雷达数据，为文件xxx。

## 结果对比
