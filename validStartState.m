function boolean = validStartState(pose, ub, vb, vj, tol)
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
%   tol is the optional scalar tolerance. If not present tolerance is 0. 
  
    position = pose(1:3);
    quat = pose(4:7);
    wRb = qt2rot(quat);
    constraints = [position + wRb * ub; cross(vb,vj)];
    if exist('tol', 'var')
        if all(constraints(:) < tol)
          % constraints values within the numerical tolerance
          boolean = true;
        else
          boolean = false;
        end
    else
        if all(constraints(:) == 0)
          boolean = true; 
        else
          boolean = false; 
        end
    end
end

