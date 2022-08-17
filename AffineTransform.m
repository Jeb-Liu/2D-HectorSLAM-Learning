function scan_world = AffineTransform(scan_body, pose)
%% 並進と回転
dx = pose(1);
dy = pose(2);
dtheta = pose(3);
%回転
ct = cos(dtheta);    
st = sin(dtheta);
R = [ct, -st; st, ct];
scan_world = scan_body*(R');
%並進
scan_world(:,1) = scan_world(:,1)+dx;
scan_world(:,2) = scan_world(:,2)+dy;