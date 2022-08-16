function pose = ScanMatch(gridMap, scan, pose, searchResolution)
distanceMatrix = gridMap.distanceMatrix;%距离矩阵
pixelSize = gridMap.pixelSize;%实际距离1m对应几个栅格单元边长
minX   = gridMap.minXY(1);%栅格地图中的最左端的横坐标(全局)
minY   = gridMap.minXY(2);%栅格地图中的最下端的纵坐标(全局)
nCols  = size(distanceMatrix, 2);%对齐距离矩阵 以%距离矩阵大小为基础
nRows  = size(distanceMatrix, 1);


%DSF(Depth First Search)
maxLoop = 50;%循环次数
maxDepth = 3;%提高分辨率的次数
loop = 0;%循环变量
depth = 0;%分辨率提高次数

pixelScan = scan/pixelSize;%将 扫描数据 实际坐标 转化为 栅格地图中的栅格坐标
bestPose  = pose;
bestScore = Inf;
dt = searchResolution(1);%x和y坐标的搜索分辨率
dr = searchResolution(3);%theta的搜索分辨率

%% 寻找最低分数
while loop < maxLoop
    noChange = true;
    %% 遍历旋转方向
    for theta = pose(3) + [-dr, 0, dr]%遍历这个三个旋转角 [旋转角-r 旋转角 旋转角+r]
        %仿射变换
        ct = cos(theta);
        st = sin(theta);
        R = [ct, st; -st, ct];
        scanPred  = pixelScan * R;%把 扫描数据(单位:栅格) 逆时针旋转theta 得到S
        
        %% 遍历平移方向
        for dx = pose(1) + [-dt, 0, dt]%遍历这三个横坐标 [预测位姿横坐标-t 预测位姿横坐标 预测位姿横坐标+t]
            xPred = round(scanPred(:,1)+(dx-minX)/pixelSize) + 1;%以栅格为单位的横坐标 (下一位姿的预测 叠加 当前位姿的扫描数据)
            for dy = pose(2) + [-dt, 0, dt]
                yPred = round(scanPred(:,2)+(dy-minY)/pixelSize) + 1;
                
                
                %筛选出落在 栅格地图内 的 遍历预测坐标点
                inRange = xPred>1 & yPred>1 & xPred<nCols & yPred<nRows;
                ix = xPred(inRange);%in range
                iy = yPred(inRange);
                %扫描点击中的栅格地图位置的一维坐标
                idx = iy + (ix-1)*nRows;
                %击中的位置转化为分数（距离）  整体分数越小 重合度越高
                scanPointsScore = distanceMatrix(idx);
                score = sum(scanPointsScore);
                
                %% 更新分数
                if score < bestScore %目的是找到最低的score(即预测栅格与当前栅格达到最高重合度)
                    noChange  = false;
                    bestPose = [dx; dy; theta];%将这个最高重合度的 预测位姿 作为最佳预测位姿
                    bestScore = score;
                end
                
            end
        end
    end
    
    % 提高分辨率
    if noChange
        dr = dr / 2;
        dt = dt / 2;
        depth = depth + 1;
        if depth > maxDepth %循环控制
            break;
        end
    end
    pose = bestPose;
    loop = loop + 1;
end