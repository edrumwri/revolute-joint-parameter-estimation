function skewR = getSkewSymmetricMatrix(v)
% getSkewSymmetricMatrix(y) generate a 3x3 skew symmetric from 3-dimensional vector v.
%   v contains 3 elements vx vy vz

skewR = [   0    -v(3)    v(2); 
          v(3)      0    -v(1); 
         -v(2)    v(1)      0 ];
end

