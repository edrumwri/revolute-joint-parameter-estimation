function [fRevolute, fDotRevolute, fDDotRevolute] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity, acceleration)
% computeRevoluteJointConstraints(ui, vi, vj, pose, velocity) computes:
%   fRevolute as the revolute joint constraints as spherical joint constraints with 
%       two additional constraints. The two additional constraints are for 
%       vector vi and vj to be parallel. 
%   fDotRevolute as the first derivative of fRevolute. 
%   fDDotRevolute as the second derivative of fRevolute.
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
%   acceleration is a 6x1 OPTIONAL vector containing:
%       translational accelerations: values 1:3 in the format \dot{vx}
%       \dot{vy} \dot{vz}
%       angular accelerations: values 4:6 in the format \dot{wx} \dot{wy}
%       \dot{wz}
%       If acceleration is not provided then fDDotSpherical will be zeros(5,1) on
%       return.

    quat = pose(4:7); % quaternion components
    wRi = qt2rot(quat);
    wRj = eye(3); % body j fixed to the world
    
    % Compute global vectors.
    viw = wRi * vi; 
    vjw = wRj * vj; 

    [v1w, v2w] = computeBasisFromAxis(viw);
    
    % Formulate the parallelism condition of the two vectors viw and vjw
    % v1iw' * vjw  = 0 and v2iw' * vjw = 0·
    revoluteParallelVect = [v1w' * vjw; v2w' * vjw];
     
    fSpherical = computeSphericalJointConstraints(ui, pose);
    fRevolute = [fSpherical; revoluteParallelVect];
    fDotRevolute = zeros(5,1);
    fDDotRevolute = zeros(5,1);
    
    if exist('velocity', 'var') 
        % the velocity is passed as an argument.
        angularVel = velocity(4:6);
    
        [fSpherical, fDotSpherical] = computeSphericalJointConstraints(ui, pose, velocity);
        % constraints for vxi (v1i and v2i) and vj vectors are in the form: 
        % f = vjw' * vxiw, where vxiw = wRi * vxi
        % \dot{f} = vjw' *  (\dot{wRi} * vxi)  
        %         = vjw' * w x (wRi * vxi) 
        %         = vjw' * w x vxiw 
        % Derivation for vxi stands for either v1i or v2i.
        
        fDotRevolute = [fDotSpherical; vjw' * cross(angularVel,v1w); vjw' * cross(angularVel,v2w)];
        
        if exist('acceleration', 'var')
            [fSpherical, fDotSpherical, fDDotSpherical] = computeSphericalJointConstraints(ui, pose, velocity, acceleration);
            angularVelDot = acceleration(4:6);
            % for the revolute constraints' derivative we derive 
            % \dot{f} = vjw' * w x (wRi * vxi) which is 
            % \ddot{f} = vjw' * \dot{w} x (wRi * vxi) 
            %          + vjw' * w x (w x (wRi * vxi))
            % Derivation for vxi stands for either v1i or v2i.
             
            fDDotRevolute = [fDDotSpherical; ...
               vjw' * cross(angularVelDot, v1w) + vjw' * cross(angularVel, cross(angularVel, v1w)); ...
               vjw' * cross(angularVelDot, v2w) + vjw' * cross(angularVel, cross(angularVel, v2w))];
        end
    end       
end
