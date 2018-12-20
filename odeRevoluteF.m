function dydt = odeRevoluteF(init, mass, Ji, ui, vi, vj)
% odeRevoluteF sets up the dynamic system for the revolute joint 
% init is the initial conditions 13 x 1 vector containing:
%   position: init(1:3) as x y z
%   orientation: init(4:7) as quaternion qw qx qy qz
%   velocity: as 
%       linear velocity: init(8:10) vx vy vz
%       angulat velocity: init(11:13) wx wy wz
%   mass represent the mass at the joint
%   Ji is the moment of inertia tensor for a hollow sphere of radius r and
%   mass m.
%   ui is a 3x1 vector from the center-of-mass of body i to the revolute 
%       joint location and expressed in the body i frame 
%   vi is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body i frame.
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the body j frame (fixed to the world).
   
    % extract the quaternion qw qx qy qz
    quat = init(4:7);
    % Linear velocity: init(8:10) vx vy vz
    linearVel = init(8:10);
    % Angular velocity wx wy wz
    angularVel = init(11:13);
    % Velocity vector: containing linear and angular 
    velocity = [linearVel; angularVel ]; 
    
    
    [F, M] = computeActingForces(mass, Ji, quat, angularVel);
   
    quatDot = computeQuatDot(quat, angularVel);
    
    pose = init(1:7);
     
    G = computeRevoluteJacobian(ui, vi, vj, pose);
    GdotV = computeRevoluteJacobianDotVAnalytical(ui, vi, vj, quat, quatDot, velocity);
    % The dynamic equations are:
    % M * \dot{vel} - G' * lambda = F
    % G * \dot{vel}  = - \dot{G} * vel
    % in matrix form we are solving for x in the system Ax = b, where
    % A = [ M -G'    x = [ \dot{vel}      b = [ F
    %       G  0 ]          lambda]                -Gdot * vel ]
    %             
    
    A = [M -G'; G zeros(5)];
    b = [F; -GdotV];
    
    x = b\A;
    
    xdot = linearVel;
    vdot = x(1:6)';
    %return the results
      dydt = [xdot; quatDot; vdot];

end

