function JacDotTimesVel = computeRevoluteJacobianDotNum(ui, vi, vj, pose, velocity, acceleration, eps)
% computeRevoluteJacobianDotNum computes numerically \dot{Jacobian} * velocity
%   Based on the formula \dot{f} = J * v, \ddot{f} = \dot{J} * v + J * \dot{v}.
%   We can then calculate \dot{J} * v =  \ddot{f} - J * \dot{v}.

    [fRevolute, fDRevolute, fDDRevolute] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity, acceleration);
    revJac = computeRevoluteJacobianNum(ui, vi, vj, pose, eps);
    JacDotTimesVel = fDDRevolute - revJac * acceleration;

end