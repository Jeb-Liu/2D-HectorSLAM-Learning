clear; close all; clc;

%% 参数
lidar.angleMin = -2.351831;% [rad] 最小扫描角
lidar.angleMax = 2.351831;% [rad] 最大扫描角
lidar.dAngle = 0.004363;% [rad] 扫描角度增量
lidar.rangeMin = 0.023;% 最近扫描距离
lidar.rangeMax = 60;% 最远扫描距离
lidar.scanAngles = (lidar.angleMin:lidar.dAngle:lidar.angleMax)';%一次扫描各线束的角度
ScanRange = 30;% [m] 采用20米内的点
pixelSize = 0.1;% [m] 栅格地图像素边长
minMove = 0.1;% [m] 移动超过0.1m 更新位姿
minRot = 0.1;% [rad] 若旋转超过0.05rad 更新位姿 
matchSteps  = [0.05; 0.05; 0.005]; % [m; m; rad]匹配分辨率

video=VideoWriter('SLAM.avi');%video
open(video);
fig1=figure(1);

%% 初始化
lidarData = load('horizental_lidar.mat');% 读取激光雷达所有数据
pose = [0; 0; 0];%初始位姿(x=0,y=0,theta=0)
path = pose;%路径=[路径,位姿]

%% 模拟循环
N = length(lidarData.times);%扫描轮数
for count = 1:2500
    
    %% 1.得到第count轮的扫描点坐标
    lastpose = pose;
    scan_body = ReadScanPoints(lidarData, count, lidar, ScanRange);%本轮扫描点的机体系坐标
    
    %% 首次扫描的点云初始化
    if count == 1
        globalPointsMap = AffineTransform(scan_body, pose);%扫描点仿射变换后放入全局点云
        isMove = true;
        continue;
    end
    
    %% 2.如果有移动 需建立新的栅格地图来匹配位姿 否则使用上回的栅格地图来与本回的扫描点匹配
    if isMove
        mapInScanRange = PointInRange(globalPointsMap, pose, scan_body);%点云内 当前扫描范围内的所有点的 全局坐标
        gridMapLow = DrawGridMap(mapInScanRange, pixelSize);%低精度栅格地图
        gridMapHigh = DrawGridMap(mapInScanRange, pixelSize/2);%高精度栅格地图
    end
    
    %% 3.预测位姿
    %预测位姿=当前位姿+(当前位姿-上一位姿)
    pose_pred = pose + (pose-lastpose);
    %使用预测位姿&低分辨率栅格地图 匹配 粗略位姿
    pose = ScanMatch(gridMapLow, scan_body, pose_pred, matchSteps);
    %使用粗略位姿&高分辨率栅格地图 匹配 位姿
    pose = ScanMatch(gridMapHigh, scan_body, pose, matchSteps/2);
    
    %% 4.如果机体 平移or旋转 超过了一定值 则判定为机体移动 记录点云 否则视为静止
    dpose = abs(lastpose - pose);%两次位姿的差
    if dpose(1)>minMove || dpose(2)>minMove || dpose(3)>minRot
        scan_world = AffineTransform(scan_body, path(:,end));
        globalPointsMap = [globalPointsMap; scan_world];%向全局图中加入新的扫描点
        isMove = true;
    else
        isMove = false;
    end
    
    %% 5.更新路径
    path = [path, pose]; %把当前位姿pose 并入路径path     

    %% 6.绘图
    if mod(count, 10) == 0%每10步画一次图
        %绘制当前全局点云 当前扫描点 当前扫描线 路径
        PlotPointsMap(count, globalPointsMap, scan_body, path);
        frame = getframe(fig1);
        writeVideo(video, frame);
    end
    
end
