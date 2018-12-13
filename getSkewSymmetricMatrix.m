function skewR = getSkewSymmetricMatrix(v)
% getSkewSymmetricMatrix(y) generate a skew 3x3 symmetric  marix out a 
% vector of v.
%   v contains 3 elements vx vy vz

skewR = [   0    -v(3)    v(2); 
          v(3)      0    -v(1); 
         -v(2)    v(1)      0 ];
end

