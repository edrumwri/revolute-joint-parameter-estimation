function dydt = odeRevoluteF(init, mass, J, ui, vi, vj)
% odeRevoluteF setups the dynamic system for the revolute joint as a
% spherical joint with 2 additional constraints
% init is the initial conditions 13 x 1 vector containing:
%   position: init(1:3) as x y z
%   orientation: init(4:7) as quaternion qw qx qy qz
%   velocity: as 
%       linear velocity: init(8:10) vx vy vz
%       angulat velocity: init(11:13) wx wy wz


    
    %extract the quaternion
    quat = init(4:7);
    % Angular velocity wx wy wz
    angularVel = init(11:13);
    % Velovity vector: containing linear velocity init(8:10) and angular velocity
    velocity = [init(8:10); angularVel ]; 
    
    
    [F, M] = computeActingForces(mass, J, quat, angularVel);
   
    quatDot = computeQuatDot(quat, angularVel);
    
    pose = init(1:7);
     
    G = computeRevoluteJacobian(ui, vi, vj, pose);
    GdotV = computeRevoluteJacobianDotVAnalytical(ui, vi, vj, quat, quatDot, velocity);
    F = computeActingForces(mass, J, quat, angularVel);
    
    % solving for x in the system Ax = b
    % A = [ M -G'    x = [ \dot{v}      b = [ F
    %       G  0 ]         \dot(w)           -Gdot*V ]
    %                      lambda]
    
    A = [M -G'; G zeros(5)];
    b = [F; -GdotV];
    
    x = b\A;
    %return the results
      dydt = [init(8) % vx
         init(9) % vy
         init(10) % vz
         quatDot(1) %\dot{qw}
         quatDot(2) %\dot{qx}
         quatDot(3) %\dot{qy}
         quatDot(4) %\dot{qz}
         x(1) %\dot{vx}
         x(2) %\dot{vy}
         x(3) %\dot{vz}
         x(4) %\dot{wx}
         x(5) %\dot{wy}
         x(6) %\dot{wz}
         ];

end

