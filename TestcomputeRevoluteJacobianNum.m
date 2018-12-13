classdef TestcomputeRevoluteJacobianNum < matlab.unittest.TestCase
    % TestcomputeRevoluteJacobianNum performs unit testing for computeRevoluteJacobianNum
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Test)
        function testComputationNoVelocity(testCase)
            % testComputationIdentity tests computation with identity rotation.
            ui = [-1 0 0]'; 
            pose = [1 0 0 1 0 0 0]';
            vi = [0 1 0]'; vj = [0 1 0]';  eps = deg2rad(1/32);
            velocity = zeros(6,1);
            actSol = computeRevoluteJacobianNum(ui, vi, vj, pose, eps) * velocity;
            [fRevolute, expSol] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
    end
end

