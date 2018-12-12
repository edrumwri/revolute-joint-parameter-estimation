function [v1i,v2i] = compute2OrtogonalVect(vi)
% compute2OrtogonalVect(vi) for a given nonzero column vector (3,1) vi compute
% vectors vi1 and vi2 form the ortogonal triad vi, v1i and v2i.
%   See Shabana book "Computational Dynamics" second ed. (2001), chapter 2 
%   on conditions of parallelism for a more detailed explanation on how
%   these vectors are computed.

    [x,i] = max(abs(vi));
    % The location of the zero element is chosen to be the same as the location 
    % of the element of vi that has the largest absolute value
    vd = ones(3, 1);
    vd(i) = 0;
    v1i = cross(vi, vd);
    v2i = cross(vi, v1i);

end