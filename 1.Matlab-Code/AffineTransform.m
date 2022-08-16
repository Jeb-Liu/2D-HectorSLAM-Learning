function scan_world = AffineTransform(scan_body, pose)
%% 仿射变换
dx = pose(1);
dy = pose(2);
dtheta = pose(3);
%旋转
ct = cos(dtheta);    
st = sin(dtheta);
R = [ct, -st; st, ct];
scan_world = scan_body*(R');
%平移
scan_world(:,1) = scan_world(:,1)+dx;
scan_world(:,2) = scan_world(:,2)+dy;