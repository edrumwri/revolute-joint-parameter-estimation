function Jspherical = computeNumJacobianSpherical(u,y, eps)
% computeNumJacobianSpherical(u,y, eps) returns a 3x6 matrix that estimates 
% numerically the jacobian for the spherical joint.
%   Uses the forward-difference formula to calculate the jacobian using 
%   function f(y) = x + R * u 

%   y is a 7x1 column vector that contains the position and orientation
%       position is y(1:3)
%       orientation as a quaternion is y(4:7)

    % perturbe y by eps
    perturbed_y = y + eps;
    
    % allocate space for forward differences calculations
    forward_f = zeros(3,7);
    f = zeros(3,7);
    % precompute the f(y)
    fi = fspherical(u,y);
     
    for i = 1:length(y)
        % Update the ith component of y for the estimation.
        estim_y = y;
        estim_y(i) = perturbed_y(i);
        % Add element to forward f calculation.
        forward_f(:,i) = fspherical(u,estim_y);
        % Add element to f calculation.
        f(:,i) = fi;
    end
    
    % extract the quaternion from y
    quat = y(4:7);
    NSpherical = computeNSpherical(quat);
    
    Jspherical = (forward_f - f)/ eps * NSpherical;
end

%% helper function that defines the function used in estimation
function out = fspherical(u,y)
% gfspherical(u,y) returns x + R * u

    x = y(1:3); %position components
    quat = y(4:7); %quaternion components
    R = quat2R(quat);
    out = x + R*u;
    
end
