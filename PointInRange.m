function mapInScanRange = PointInRange(pointsMap, pose, scan_body)
% 並進と回転
scan_world = AffineTransform(scan_body, pose);
% この回のスキャンした点が世界座標系にある範囲
minX = min(scan_world(:,1) - 1);
minY = min(scan_world(:,2) - 1);
maxX = max(scan_world(:,1) + 1);
maxY = max(scan_world(:,2) + 1);
% point cloud map の範囲内の点を取り出す
inRange = pointsMap(:,1)>minX   &   pointsMap(:,1)<maxX  &  pointsMap(:,2)>minY  &  pointsMap(:,2)<maxY;
% point cloud mapから取り出したこの回のscan point
mapInScanRange = pointsMap(inRange, :);