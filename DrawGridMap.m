function gridMap = DrawGridMap(scan_world, pixelSize)

minXY = min(scan_world) - 3 * pixelSize;%一番左下の座標 3pixel拡大
maxXY = max(scan_world) + 3 * pixelSize;%一番右上の座標 3pixel拡大

gridSize = round((maxXY - minXY) / pixelSize) + 1;%pixelマップの大きさ(pixel数)
N = size(scan_world, 1);%スキャンした数
scan_zerofixed = round( (scan_world-repmat(minXY, N, 1)) / pixelSize ) + 1;%1番左下の座標を[1,1]にする

idx = (scan_zerofixed(:,1)-1)*gridSize(2) + scan_zerofixed(:,2);%占用したpixelの2次元座標を1次元座標にする
grid  = false(gridSize(2), gridSize(1));%空pixelマップ
grid(idx) = true;%占用したらture

%% point cloud mapに書き込み
gridMap.occu = grid;%point cloud map
gridMap.distanceMatrix = bwdist(grid);%各点と一番近い0との距離の和(テ点数)
gridMap.pixelSize = pixelSize;%1つのpixelの実際の辺長
gridMap.minXY = minXY;%世界座標系における1番左下の点の座標