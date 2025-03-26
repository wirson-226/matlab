function q = RotToQuat(R)
% ROTTOQUAT Converts a Rotation matrix into a Quaternion
% Supports World ENU to Body RFS coordinate system
tr = R(1,1) + R(2,2) + R(3,3);
if (tr > 0)
 S = sqrt(tr+1.0) * 2; % S=4*qw
 qw = 0.25 * S;
 qx = (R(3,2) - R(2,3)) / S;
 qy = (R(1,3) - R(3,1)) / S;
 qz = (R(2,1) - R(1,2)) / S;
elseif ((R(1,1) > R(2,2)) && (R(1,1) > R(3,3)))
 S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx
 qw = (R(3,2) - R(2,3)) / S;
 qx = 0.25 * S;
 qy = (R(1,2) + R(2,1)) / S;
 qz = (R(1,3) + R(3,1)) / S;
elseif (R(2,2) > R(3,3))
 S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*qy
 qw = (R(1,3) - R(3,1)) / S;
 qx = (R(1,2) + R(2,1)) / S;
 qy = 0.25 * S;
 qz = (R(2,3) + R(3,2)) / S;
else
 S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*qz
 qw = (R(2,1) - R(1,2)) / S;
 qx = (R(1,3) + R(3,1)) / S;
 qy = (R(2,3) + R(3,2)) / S;
 qz = 0.25 * S;
end
q = [qw;qx;qy;qz];
q = q*sign(qw);
end

function R = QuatToRot(q)
% QuatToRot Converts a Quaternion to Rotation matrix
q = q./sqrt(sum(q.^2));
qahat(1,2) = -q(4);
qahat(1,3) = q(3);
qahat(2,3) = -q(2);
qahat(2,1) = q(4);
qahat(3,1) = -q(3);
qahat(3,2) = q(2);
R = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;
end

function [phi, theta, psi] = RotToRPY_ENU_to_RFS(R)
% RotToRPY Convert rotation matrix from World ENU to Body RFS 
% phi: Roll (rotation around X-axis in body frame)
% theta: Pitch (rotation around Y-axis in body frame)
% psi: Yaw (rotation around Z-axis in body frame)

% Note: This implements YXZ rotation order which is equivalent to ZXY
phi = asin(R(2,3));  % sin(pitch)
theta = atan2(-R(2,1), R(2,2));  % roll
psi = atan2(-R(1,3), R(3,3));   % yaw
end

function R = RPYtoRot_ENU_to_RFS(phi, theta, psi)
% RPYtoRot Convert RPY angles to rotation matrix (World ENU to Body RFS)
% Uses YXZ (equivalent to ZXY) rotation order

R = [cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta), ...
     -cos(phi)*sin(psi), ...
     cos(psi)*sin(theta) - sin(psi)*cos(theta)*sin(phi); ...
     sin(psi)*cos(theta) - cos(psi)*sin(phi)*sin(theta), ...
     cos(phi)*cos(psi), ...
     sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi); ...
     -cos(phi)*sin(theta), ...
     sin(phi), ...
     cos(phi)*cos(theta)];
end

function R_world = ConvertBodyToWorld(R_body)
% Convert body (RFS) rotation matrix to world (ENU) frame
R_world = [0 1 0;   % Body x -> World y
           0 0 1;   % Body y -> World z
           1 0 0];  % Body z -> World x
R_world = R_world * R_body * R_world';
end

function R_body = ConvertWorldToBody(R_world)
% Convert world (ENU) rotation matrix to body (RFS) frame
R_body = [0 0 1;   % World x -> Body z
          1 0 0;   % World y -> Body x
          0 1 0];  % World z -> Body y
R_body = R_body * R_world * R_body';
end