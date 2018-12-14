function [fRevolute, fDotRevolute] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity)
% computeRevoluteJointConstraints(ui, vi, vj, pose, velocity) computes:
%   fRevolute as the revolute joint constraints as spherical joint constraints with 
%       two additional constraints. The two additional constraints are for 
%       vector vi and vj to be parallel. 
%   fDotRevolute as the first derivative of fRevolute using the analytical derivation.
%
%   ui is a 3x1 vector from the center-of-mass of body i to the revolute 
%       joint location and expressed in the body i frame 
%   vi is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body i frame.
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body j frame (fixed to the world).
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   velocity is a 6x1 OPTIONAL vector containing:
%       velocity: values 1:3 in the format vx vy vz
%       angular velocity: values 4:6 in the format wx wy wz
%       If velocity is not provided then fDotRevolute will be zeros(5,1) on
%       return.

    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    wRj = eye(3); % body j fixed to the world
    
    % Compute global vectors.
    viw = wRi * vi; 
    vjw = wRj * vj; 

    [v1iw, v2iw] = computeBasisFromAxis(viw);
    
    % Formulate the parallelism condition of the two vectors viw and vjw
    % v1iw' * vjw  = 0 and v2iw' * vjw = 0·
    revoluteParallelVect = [v1iw' * vjw; v2iw' * vjw];
     
    fSpherical = computeSphericalJointConstraints(ui, pose);
    fRevolute = [fSpherical; revoluteParallelVect];
    
    if exist('velocity','var') 
        % the velocity is passed as an argument.
        angularVelTensor = getSkewSymmetricMatrix(velocity(4:6));
    
        [fSpherical fDotSpherical] = computeSphericalJointConstraints(ui, pose, velocity);
        % constraints for vi and vj vectors are in the form: f = v1iw' * vjw , then 
        % df/dt =  v1iw' * angularVelTensor' * vjw, angularVelTensor' = - angularVelTensor
        fDotRevolute = [fDotSpherical; v1iw' * angularVelTensor' *  vjw; v2iw' * angularVelTensor' *  vjw];
    else
        fDotRevolute = zeros(5,1);
    end
        
end
