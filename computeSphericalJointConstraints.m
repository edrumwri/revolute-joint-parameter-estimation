function [fSpherical, fDotSpherical, fDDotSpherical] = computeSphericalJointConstraints(ui, pose, velocity, acceleration)
% computeSphericalJointConstraints(ui, pose, velocity) computes:
%   fSpherical: the spherical joint constraints defined as x + R * u = 0.
%   fDotSpherical: as the first derivative of fSpherical using the analytical derivation.
%   fDDotSpherical: as the second derivative of fSpherical using the analytical derivation.
%
%   ui is a 3x1 vector from the center-of-mass of body i to the revolute 
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   velocity is a 6x1 OPTIONAL vector containing:
%       velocity: values 1:3 in the format vx vy vz
%       angular velocity: values 4:6 in the format wx wy wz
%       If velocity is not provided then fDotSpherical will be zeros(3,1) on
%       return.
%   acceleration is a 6x1 OPTIONAL vector containing:
%       translational accelerations: values 1:3 in the format \dot{vx}
%       \dot{vy} \dot{vz}
%       angular accelerations: values 4:6 in the format \dot{wx} \dot{wy}
%       \dot{wz}
%       If acceleration is not provided then fDDotSpherical will be zeros(3,1) on
%       return.

    x = pose(1:3); % position components
    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    fSpherical = x + wRi * ui;
    
    if exist('velocity', 'var')
        % the velocity is passed as an argument.
        vel = velocity(1:3);
        angularVelTensor = getSkewSymmetricMatrix(velocity(4:6));
    
        % f = x + R * u, df/dt = dx/xt + dR/dt * u 
        % dx/dt = vel, dR/dt * u = angularVelTensor * R * u  
        % therfore df/dt = vel + angularVelTensor * R * u 
        fDotSpherical = vel + angularVelTensor * wRi * ui;    
        
        if exist('acceleration', 'var')
            % for the revolute constraints derivative we derive 
            % \dot{f} = vel + w x (wRi * ui) , which is 
            % \ddot{f} = \dot{vel} + \dot{w} x (wRi * ui)  
            %           + w x (w x (wRi * ui)).
            uw = wRi * ui;
            vDot = acceleration(1:3);
            angularVel = velocity(4:6);
            angularVelDot = acceleration(4:6);
           
            fDDotSpherical = vDot + cross(angularVelDot, uw) + cross(angularVel, cross(angularVel, uw));
        
        else
            fDDotSpherical = zeros(3,1);
        end
    else
        fDotSpherical = zeros(3,1);
    end
    
    
    
end

