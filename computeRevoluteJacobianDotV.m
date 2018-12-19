function JacDotTimesVel = computeRevoluteJacobianDotV(ui, vi, vj, pose, velocity, acceleration)
% computeRevoluteJacobianDotV computes numerically \dot{Jacobian} * velocity
%   Based on the formula \dot{f} = J * v, \ddot{f} = \dot{J} * v + J * \dot{v}.
%   We can then calculate \dot{J} * v =  \ddot{f} - J * \dot{v}.

    [fRevolute, fDRevolute, fDDRevolute] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity, acceleration);
    revJac = computeRevoluteJacobian(ui, vi, vj, pose(4:7));
    JacDotTimesVel = fDDRevolute - revJac * acceleration;

end