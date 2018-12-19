function JSpherical = computeSphericalJacobian(ui, quat)
% computeSphericalJacobian analytical computation of the jacobian
% for the spherical joint.
%   The spherical constraint is f = x + wRi * ui. 
%   \dot{f} = \dot{x} + \dot{wRi} * ui
%           = \dot{x} + skew(w) * (wRi * ui) , using skew(a) * b = -b * skew(a)
%           = \dot{x} - (wRi * ui) * skew(w) , using a * skew(b) = skew(a) * b
%           = \dot{x} - skew(wRi * ui) * w
%   by separating the velocity terms \dot{x} and w we get 
%   JSpherical = [eye(3) -skew(wRi * ui)]

  wRi = qt2rot(quat);
  JSpherical = [eye(3) -getSkewSymmetricMatrix(wRi * ui)];

end

