function mapInScanRange = PointInRange(pointsMap, pose, scan_body)
% 仿射变换
scan_world = AffineTransform(scan_body, pose);
% 本轮扫描点在全局坐标的范围
minX = min(scan_world(:,1) - 1);
minY = min(scan_world(:,2) - 1);
maxX = max(scan_world(:,1) + 1);
maxY = max(scan_world(:,2) + 1);
% 提取范围内点云
inRange = pointsMap(:,1)>minX   &   pointsMap(:,1)<maxX  &  pointsMap(:,2)>minY  &  pointsMap(:,2)<maxY;
% 从全局地图中提取到的当前扫描点
mapInScanRange = pointsMap(inRange, :);