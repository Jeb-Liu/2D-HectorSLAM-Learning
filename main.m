clear; close all; clc;

%% Lidar仕様
lidar.angleMin = -2.351831;% [rad] スキャン角min
lidar.angleMax = 2.351831;% [rad] スキャン角max 
lidar.dAngle = 0.004363;% [rad] 2つのスキャンのなす角
lidar.rangeMin = 0.023;% スキャン距離min
lidar.rangeMax = 60;% スキャン距離max
lidar.scanAngles = (lidar.angleMin:lidar.dAngle:lidar.angleMax)';%1回のスキャンにおける各レーザーの角度
ScanRange = 30;% [m] 30m以内のデータを使う
pixelSize = 0.1;% [m] pixelの実際の辺長
minMove = 0.1;% [m] 移動距離が0.1mを超えると 位置姿勢を更新
minRot = 0.1;% [rad] 回転が0.1radを超えると 位置姿勢を更新
matchSteps  = [0.05; 0.05; 0.005]; % [m; m; rad]matchingの解像度

video=VideoWriter('SLAM.avi');%save video
open(video);
fig1=figure(1);

%% 初始化
lidarData = load('horizental_lidar.mat');% lidarのデータを読み込む
pose = [0; 0; 0];%位置姿勢初期値(x=0,y=0,theta=0)
path = pose;%経路=[経路,位置姿勢]

%% 模拟循环
N = length(lidarData.times);%スキャンの回数
for count = 1:2500
    
    %% 1.第count回目のスキャンした点の座標
    lastpose = pose;
    scan_body = ReadScanPoints(lidarData, count, lidar, ScanRange);%各点の機体系座標
    
    %% 第1回目のcloud mapを初期化
    if count == 1
        globalPointsMap = AffineTransform(scan_body, pose);%スキャンした点をcloud mapに書き込む
        isMove = true;
        continue;
    end
    
    %% 2.移動したら、新しいpixel mapを作って位置姿勢をマッチング、それではないと前回のpixel mapを使って位置姿勢をマッチング
    if isMove
        mapInScanRange = PointInRange(globalPointsMap, pose, scan_body);%cloud mapにおける 範囲内のすべての点の世界座標
        gridMapLow = DrawGridMap(mapInScanRange, pixelSize);%低解像度pixel map
        gridMapHigh = DrawGridMap(mapInScanRange, pixelSize/2);%低解像度pixel map
    end
    
    %% 3.位置姿勢を予測
    %予測位置姿勢=位置姿勢+(位置姿勢-前回の位置姿勢)
    pose_pred = pose + (pose-lastpose);
    %予測位置姿勢　と　低解像度pixel map　を使て　大体の位置姿勢　をマッチング
    pose = ScanMatch(gridMapLow, scan_body, pose_pred, matchSteps);
    %大体の位置姿勢　と　高解像度pixel map　を使て　位置姿勢　をマッチング
    pose = ScanMatch(gridMapHigh, scan_body, pose, matchSteps/2);
    
    %% 4.機体の並進と回転運動が一定値を超える場合、移動しているとする、それではないと静止とする
    dpose = abs(lastpose - pose);%位置姿勢の差
    if dpose(1)>minMove || dpose(2)>minMove || dpose(3)>minRot
        scan_world = AffineTransform(scan_body, path(:,end));
        globalPointsMap = [globalPointsMap; scan_world];%cloud mapに点を書き込む
        isMove = true;
    else
        isMove = false;
    end
    
    %% 5.経路を更新
    path = [path, pose]; %経路と今の位置姿勢    

    %% 6.plot and video
    if mod(count, 10) == 0%10回ごとplotする
        %plotするもの：point cloud map、今のスキャンの点、レーザ、経路
        PlotPointsMap(count, globalPointsMap, scan_body, path);
        frame = getframe(fig1);
        writeVideo(video, frame);
    end
    
end
close(video);