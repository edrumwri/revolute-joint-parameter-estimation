function J_B = getHollowSphereInertiaTensor(mass, sphere_radius)
% getHollowSphereInertiaTensor(mass, sphere_radius) function that defines 
% the inertia tensor for a 3D hollow sphere of mass "mass" and radius "sphere_radius".

J_B = eye(3) * 2/3 * mass * sphere_radius^2;
end