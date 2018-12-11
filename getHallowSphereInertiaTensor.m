function I = getHallowSphereInertiaTensor(m,r)
%getHallowSphereInertiaTensor function that defines the inertia tensor for
%a 3D hallow sphere of mass m and radius r

I = diag([2/3*m*r^2, 2/3*m*r^2, 2/3*m*r^2]);

end