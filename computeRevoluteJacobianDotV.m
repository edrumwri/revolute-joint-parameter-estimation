function JacDotTimesVel = computeRevoluteJacobianDotV(ui, vi, vj, pose, velocity, acceleration)
% computeRevoluteJacobianDotV computes numerically \dot{Jacobian} * velocity
%   Based on the formula:
%   \dot{f}     = J * v, 
%   \ddot{f}    = \dot{J} * v + J * \dot{v}, we than have
%   \dot{J} * v =  \ddot{f} - J * \dot{v}.

    [fRevolute, fDRevolute, fDDRevolute] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity, acceleration);
    quat = pose(4:7);
    revJac = computeRevoluteJacobian(ui, vi, vj, quat);
    JacDotTimesVel = fDDRevolute - revJac * acceleration;

end