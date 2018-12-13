function R = qt2rot(q)
% qt2rot(q) converts a quaternion q to a 3x3 rotation matrix. 


  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  q4 = q(4);

R = [ 1-2*(q3^2 + q4^2)   2*(q2*q3 - q4*q1)   2*(q2*q4 + q3*q1)
      2*(q2*q3 + q4*q1)   1-2*(q2^2 + q4^2)   2*(q3*q4 - q2*q1)
      2*(q2*q4 - q3*q1)   2*(q3*q4 + q2*q1)   1-2*(q2^2 + q3^2) ];
end
  