function JSpherical = computeSphericalJacobian(ui, quat)
% computeSphericalJacobian analytical computation of the jacobian
% for the spherical joint.
%   The spherical constraint is f = x + wRi * ui. 
%   \dot{f} = \dot{x} + \dot{wRi} * ui
%           = \dot{x} + w x (wRi * ui) 
%           = \dot{x} - (wRi * ui) x w 
%           = \dot{x} - skew(wRi * ui) * w
%   by separating the velocity terms \dot{x} and w we get 
%   JSpherical = [eye(3) -skew(wRi * ui)]

  wRi = qt2rot(quat);
  JSpherical = [eye(3) -getSkewSymmetricMatrix(wRi * ui)];

end

