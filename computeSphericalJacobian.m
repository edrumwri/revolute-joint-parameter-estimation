function JSpherical = computeSphericalJacobian(ui, quat)
% computeSphericalJacobian analytical computation of the jacobian
% for the spherical joint.
% JSpherical = [eye(3) -skew(wRi * ui)]

  wRi = qt2rot(quat);
  JSpherical = [eye(3) -getSkewSymmetricMatrix(wRi * ui)];

end

