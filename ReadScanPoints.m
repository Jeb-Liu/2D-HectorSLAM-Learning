function points = ReadScanPoints(lidarData, n, lidar, ScanRange)
    
    angles = lidar.scanAngles;%各レーザの角度
    ranges = lidarData.ranges(n, :)';%各レーザの距離
    
    %範囲外のデータを消去
    maxRange = min(lidar.rangeMax, ScanRange);
    notInRange = ranges<lidar.rangeMin  |  ranges>maxRange;
    angles(notInRange) = [];
    ranges(notInRange) = [];
    
    % 極座標から直交座標に変換
    [x, y] = pol2cart(angles, ranges);
    points = [x, y];%この回のスキャンした点の機体系座標
end