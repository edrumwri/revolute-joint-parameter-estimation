function eDot = computeQuatDot(quat, angularVelocity)
% computeQuatDot Calculates the derivative of the quaternion
%   Based on Nikravesh formula 6.104 \dot{quat} = 1/2 * G' * w

  G = computeNikraveshG(quat);
  eDot = 0.5 * G' * angularVelocity;
end

