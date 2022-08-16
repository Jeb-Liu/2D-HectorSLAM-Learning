function gridMap = DrawGridMap(scan_world, pixelSize)

minXY = min(scan_world) - 3 * pixelSize;%最左最下的坐标 并拓展3格
maxXY = max(scan_world) + 3 * pixelSize;%最右最上的坐标 并拓展3格

gridSize = round((maxXY - minXY) / pixelSize) + 1;%栅格地图的大小(格数)
N = size(scan_world, 1);%扫描点点的个数
scan_zerofixed = round( (scan_world-repmat(minXY, N, 1)) / pixelSize ) + 1;%纠正扫描点零偏(使最左最下为[1,1])

idx = (scan_zerofixed(:,1)-1)*gridSize(2) + scan_zerofixed(:,2);%把被占用的栅格的二维坐标转化为一维坐标
grid  = false(gridSize(2), gridSize(1));%空的栅格地图
grid(idx) = true;%被占用为ture

%% 写入地图 
gridMap.occu = grid;%栅格地图
gridMap.distanceMatrix = bwdist(grid);%各个格内元素为离最近的0元素的距离(分数)
gridMap.pixelSize = pixelSize;%栅格单元边长对应的实际长度
gridMap.minXY = minXY;%栅格地图的x最小值和y最小值构成的向量的全局坐标