function JRevNum = computeRevoluteJacobianNum(ui, vi, vj, pose, eps)
% computeRevoluteJacobianNum(ui, vi, vj, pose, eps) returns a 5x6 matrix that estimates 
% numerically the jacobian for the spherical joint.
%   Uses the forward-difference formula to calculate the jacobian using 
%   computeRevoluteJointConstraints function
%   ui is a 3x1 vector representing local position vector on body i 
%   vi is a 3x1 unit vector for the revolute joint in the body i frame.
%   vj is a 3x1 unit vector for the revolute joint in the body j frame (fixed to the world).
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   eps is the epsilon used in the forward calculation.

    % perturbe pose by eps
    posePerturbed = pose + eps;
    
    % Allocate space for forward differences calculations.
    col = length(pose);
    fForward = zeros(5, col);
    f = zeros(5, col);
    % Precompute the value for the constraints with no perturbance.
    fi = computeRevoluteJointConstraints(ui, vi, vj, pose);
    
    for i = 1:col
        % Update the ith component of pose for the estimation.
        poseForward = pose;
        poseForward(i) = posePerturbed(i);
        % Add result to forward f calculation.
        fForward(:, i) = computeRevoluteJointConstraints(ui, vi, vj, poseForward);
        % Add element to f calculation.
        f(:, i) = fi;
    end
    
    % Extract the quaternion from pose.
    quat = pose(4:7);
    N = computeN(quat);
    
    JRevNum = (fForward - f)/ eps * N;    
end

