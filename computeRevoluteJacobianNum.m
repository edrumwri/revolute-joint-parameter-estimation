function JRevNum = computeRevoluteJacobianNum(ui, vi, vj, pose, eps)
% computeRevoluteJacobianNum(ui, vi, vj, pose, eps) returns a 5x6 matrix that estimates 
% numerically the jacobian for the revolute joint.
%   Uses the forward-difference formula to calculate the jacobian using 
%   computeRevoluteJointConstraints function
%   ui is a 3x1 vector from the center-of-mass of body i to the revolute 
%       joint location and expressed in the body i frame 
%   vi is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body i frame.
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body j frame (fixed to the world).
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   eps is the epsilon used in the forward calculation.

    
    % Allocate space for forward differences calculations.
    col = length(pose);
    fForward = zeros(5, col);
    
    % Precompute the value for the constraints with no perturbance.
    fCurrent = computeRevoluteJointConstraints(ui, vi, vj, pose);
    
    for i = 1:col
        % Update the ith component of pose for the estimation.
        poseForward = pose;
        poseForward(i) =  poseForward(i) + eps;
        % Add result to forward f calculation.
        fForward(:, i) = computeRevoluteJointConstraints(ui, vi, vj, poseForward);
    end
    
    % Extract the quaternion from pose.
    quat = pose(4:7);
    N = computeN(quat);
    
    % JRevNum * v = df/dq * dq/dt and dq/dt = N*v, therfore 
    % JRevNum * v = df/dq * N * v, making JRevNum = df/dq * N
    JRevNum = (fForward - fCurrent) / eps * N;    
end

