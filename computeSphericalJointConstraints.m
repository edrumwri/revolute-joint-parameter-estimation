function RevConstr = computeSphericalJointConstraints(u, y)
% computeSphericalJointConstraints(u, y) computes the spherical joint
% constraints

    x = y(1:3); %position components
    quat = y(4:7); %quaternion components
    wRi = quat2R(quat);
    RevConstr = x + wRi*u;
    
end

