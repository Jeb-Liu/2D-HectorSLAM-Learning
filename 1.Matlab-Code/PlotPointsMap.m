function PlotPointsMap(count, pointsMap, scan_body, path)
        fig1=figure(1); clf; hold on; axis equal;
        disp(['Number of scan : ', num2str(count)]);
        scan_world = AffineTransform(scan_body, path(:,end));
        plot(pointsMap(:,1), pointsMap(:,2), '*', 'MarkerSize', 1, 'color', 'black');%画全局地图点集
        plot(scan_world(:,1),  scan_world(:,2),  '.', 'MarkerSize', 1, 'color', 'c');%画当前的扫描点图
        plot(path(1,:),  path(2,:), 'color', 'r');%画路径
        for i = 1:20:length(scan_world)%lidar扫描范围
            line([path(1,end), scan_world(i,1)], [path(2,end), scan_world(i,2)],'Color','g','LineStyle','-.');
        end
        drawnow;
end

