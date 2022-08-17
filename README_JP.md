# 2D-HectorSLAM-Learning
## LIDARを搭載したローバが取得したデータを使用し、周辺を再現して経路を求める。
input：LIDARの水平方向の時系列データ
output：ローバの経路、point cloud map

## データソース&フォーマット
データは2D Cartographer Backpack – Deutsches Museum
https://google-cartographer-ros.readthedocs.io/en/latest/data.html#d-cartographer-backpack-deutsches-museum

データのすべての組成は、水平方向と鉛直方向のLIDARの時系列データ、加速度センサの時系列データ。
今回のプログラムは、水平方向のLIDARの時系列データのみ使用した(.mat)。


## 结果对比
