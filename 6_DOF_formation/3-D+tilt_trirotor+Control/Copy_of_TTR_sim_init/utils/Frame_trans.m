function q = RotToQuat(R)
%ROTTOQUAT Converts a Rotation matrix into a Quaternion
%   written by Daniel Mellinger
%   from the following website, deals with the case when tr<0
%   http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

%takes in W_R_B rotation matrix

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
%   QuatToRot Converts a Quaternion to Rotation matrix
%   written by Daniel Mellinger

% normalize q
q = q./sqrt(sum(q.^2));

qahat(1,2) = -q(4);
qahat(1,3) = q(3);
qahat(2,3) = -q(2);
qahat(2,1) = q(4);
qahat(3,1) = -q(3);
qahat(3,2) = q(2);

R = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;

end


function [phi,theta,psi] = RotToRPY_ZXY(R)
%   RotToRPY_ZXY Extract Roll, Pitch, Yaw from a world-to-body Rotation Matrix
%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose the matrix if you have a body to world [wRb] such
%   that [wP] = [wRb] * [bP], where [bP] is a point in the body frame and
%   [wP] is a point in the world frame
%   written by Daniel Mellinger
%   bRw = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),
%           cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),
%          -cos(phi)*sin(theta)]
%         [-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi)]
%         [ cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
%           sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
%           cos(phi)*cos(theta)]

phi = asin(R(2,3));
psi = atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi));
theta = atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));

end


function R = RPYtoRot_ZXY(phi,theta,psi)
%   RPYtoRot_ZXY Converts roll, pitch, yaw to a body-to-world Rotation matrix
%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose this matrix to get the body to world [wRb] such that
%   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
%   is a point in the world frame
%   written by Daniel Mellinger
%

R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), ...
     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), ...
     -cos(phi)*sin(theta); ...
     -cos(phi)*sin(psi),...
     cos(phi)*cos(psi), ...
     sin(phi);...
     cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),...
     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),...
     cos(phi)*cos(theta)];

end
