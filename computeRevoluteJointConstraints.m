function cross = computeRevoluteJointConstraints(ui, uj, quat)
% computeRevoluteJointConstraints(ui, uj, quat) returns the cross product of
% vectors ui and uj.
%   vector ui is in the body i frame .
%   vector uj is in the global frame (fixed to the world).

   wRi = quat2R(quat);
   wRj = eye(3); % the 2 body (j) is attached to the world
   cross = (wRi*ui)' * (wRj*uj);
end

