function pose = ScanMatch(gridMap, scan, pose, searchResolution)
distanceMatrix = gridMap.distanceMatrix;%距離行列
pixelSize = gridMap.pixelSize;
minX   = gridMap.minXY(1);%pixel mapの一番左のデータの世界座標
minY   = gridMap.minXY(2);%pixel mapののデータの世界座標
nCols  = size(distanceMatrix, 2);
nRows  = size(distanceMatrix, 1);


%DSF(Depth First Search)
maxLoop = 50;%loop回数
maxDepth = 3;%解像度の最大増加回数
loop = 0;
depth = 0;

pixelScan = scan/pixelSize;%座標をpixel mapの座標へ
bestPose  = pose;
bestScore = Inf;
dt = searchResolution(1);%xとyの座標の検索解像度
dr = searchResolution(3);%thetaの検索解像度

%% 一番低い点数を探す
while loop < maxLoop
    noChange = true;
    %% 回転方向を巡回する
    for theta = pose(3) + [-dr, 0, dr]%3つの回転角を巡回 [回転角-r 回転角 回転角+r]
        %回転と並進
        ct = cos(theta);
        st = sin(theta);
        R = [ct, st; -st, ct];%回転行列
        scanPred  = pixelScan * R;
        
        %% 遍历平移方向
        for dx = pose(1) + [-dt, 0, dt]%3つのx座標を巡 [予測x座標-t 予測x座標 予測x座標+t]
            xPred = round(scanPred(:,1)+(dx-minX)/pixelSize) + 1;%pixel mapにおけるx座標
            for dy = pose(2) + [-dt, 0, dt]
                yPred = round(scanPred(:,2)+(dy-minY)/pixelSize) + 1;%pixel mapにおけるy座標
                
                
                %pixel map以内　の　巡回予測座標　を取り出す
                inRange = xPred>1 & yPred>1 & xPred<nCols & yPred<nRows;
                ix = xPred(inRange);%in range
                iy = yPred(inRange);
                %扫描点击中的栅格地图位置的一维坐标　scan pointがpixel mapを占用した位置の1次元座標
                idx = iy + (ix-1)*nRows;
                %占用した位置を点数（距離）へ  点数が小さいほど重合度が高い
                scanPointsScore = distanceMatrix(idx);
                score = sum(scanPointsScore);
                
                %% 点数を更新
                if score < bestScore %目的は一番低い点数(予測pixel mapと現在のpixel mapが一番重合度高い時)
                    noChange  = false;
                    bestPose = [dx; dy; theta];%重合度が一番高いの予測位置姿勢を 作为一番良い位置姿勢
                    bestScore = score;
                end
                
            end
        end
    end
    
    % 解像度を高める
    if noChange
        dr = dr / 2;
        dt = dt / 2;
        depth = depth + 1;
        if depth > maxDepth %巡回停止条件
            break;
        end
    end
    pose = bestPose;
    loop = loop + 1;
end