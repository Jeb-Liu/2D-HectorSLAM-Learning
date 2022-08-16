function points = ReadScanPoints(lidarData, n, lidar, ScanRange)
    
    angles = lidar.scanAngles;%读取各扫描角
    ranges = lidarData.ranges(n, :)';%读取各扫描角的扫描点距离
    
    %删除范围外的点
    maxRange = min(lidar.rangeMax, ScanRange);
    notInRange = ranges<lidar.rangeMin  |  ranges>maxRange;
    angles(notInRange) = [];
    ranges(notInRange) = [];
    
    % 从极坐标转换为笛卡尔坐标
    [x, y] = pol2cart(angles, ranges);
    points = [x, y];%时刻n时所有数据点的body fixed坐标
end