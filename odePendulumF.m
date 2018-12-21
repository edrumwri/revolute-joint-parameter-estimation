function dydt = odePendulumF(init, mass, Jb, ub, vi, vj)
% odePendulumF sets up the dynamic system for a pendulum constrained in
%   absolute coordinates.
% init is the initial state, 13 x 1 vector, containing:
%   position: init(1:3) as x y z
%   orientation: init(4:7) as quaternion qw qx qy qz
%   velocity: as 
%       linear velocity: init(8:10) vx vy vz
%       angulat velocity: init(11:13) wx wy wz
%   mass represents the mass of the body.
%   Jb is the moment of inertia tensor, expressed in the pendulum bob frame
%   for a hollow sphere of radius r and mass m.
%   ub is a 3x1 vector from the center-of-mass of the bob to the revolute 
%       joint location and expressed in the bob frame. 
%   vb is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the bob frame. 
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the j frame (fixed to the world).
   
    % extract the quaternion qw qx qy qz
    quat = init(4:7);
    % Linear velocity: init(8:10) vx vy vz
    linearVel = init(8:10);
    % Angular velocity wx wy wz
    angularVel = init(11:13);
    % Velocity vector: containing linear and angular 
    velocity = [linearVel; angularVel ]; 
    
    
    [F, M] = computeActingForcesAndGeneralizedInertia(mass, Jb, quat, angularVel);
   
    quatDot = computeQuatDot(quat, angularVel);
    
    pose = init(1:7);
     
    G = computeRevoluteJacobian(ub, vi, vj, pose);
    GdotVel = computeRevoluteJacobianDotVAnalytical(ub, vi, vj, quat, quatDot, velocity);
    % The dynamic equations are:
    % M * \dot{velocity} - G' * lambda = F
    % G * \dot{velocity}  = - \dot{G} * vel
    % in matrix form we are solving for x in the system Ax = b, where
    % A = [ M -G'    x = [ \dot{velocity}      b = [ F
    %       G  0 ]          lambda]                -Gdot * velocity ]
    %             
    
    A = [M -G'; G zeros(5)];
    b = [F; -GdotVel];
    
    x = b\A;
    
    xdot = linearVel;
    vdot = x(1:6)';
    %return the results
      dydt = [xdot; quatDot; vdot];

end

