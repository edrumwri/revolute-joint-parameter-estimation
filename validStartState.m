function boolean = validStartState(pose, ub, vb, vj)
% validStartState tests if the inital configuration of the pendulum is valid. 
%       retuns boolean true or false. 
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   ub is a 3x1 vector from the center-of-mass of the bob to the revolute 
%       joint location and expressed in the bob frame. 
%   vb is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the bob frame. 
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the j frame (fixed to the world).
  
    position = pose(1:3);
    quat = pose(4:7);
    wRb = qt2rot(quat);
    constraints = [position + wRb * ub; cross(vb,vj)];
    if all(constraints(:) == 0)
      boolean = 1; 
    else
      boolean = 0;
    end
end

