
mass = 1;
sphere_radius = 0; % when radius is zero we 

% Moment of inertia tensor for a hollow sphere of radius r and mass m
Ji = getHollowSphereInertiaTensor(mass, sphere_radius);
%   ub is a 3x1 vector from the center-of-mass of the bob to the revolute 
%       joint location and expressed in the bob frame. 
%   vb is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the bob frame. 
%   vj is a 3x1 unit vector that points along the axis of the revolute joint 
%       expressed in the j frame (fixed to the world).

ub = [-1; 0; 0];
vb = [0; 0; 1];
vj = [0; 0; 1];

% Variable that defines the initial state of the pendulum. If not at rest 
% the pendulum starts from a horizontal position (90 degrees with the
% vertical).
atRest = 1; 

if atRest
   position = [0 -1 0]';
   orientation =  qt([0 0 1], -pi/2);
   linearVel = zeros(3,1);
   angularVel = zeros(3,1); 
else
    position = [1 0 0]';
    orientation = [1 0 0 0]';
    linearVel = zeros(3,1);
    angularVel = [0 0 1]';
end

% y0rev is the initial conditions 13 x 1 vector containing:
%   position: y0rev(1:3) as x y z
%   orientation: y0rev(4:7) as quaternion qw qx qy qz
%   velocity: as 
%       linear velocity: y0rev(8:10) vx vy vz
%       angulat velocity: y0rev(11:13) wx wy wz

y0rev = [position; orientation; linearVel; angularVel];

% Test constraints.
pose = y0rev(1:7);
if validStartState(pose, ub, vb, vj, 1e-16)  
    tspan = [0;10];
    [t,y3Drev] = ode45(@(t,y)odePendulumF(y, mass, Ji, ub, vb, vj), tspan, y0rev);
    % Plot the resulting motion of the joint
    plot(y3Drev(:,1),y3Drev(:,2))
    xlabel('X') 
    ylabel('Y') 
    axis equal;
else
    error('simulatePendulum:ConstraintsNotMeet','Initial conditions are not valid.'); 
end