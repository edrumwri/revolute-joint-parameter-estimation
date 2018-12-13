classdef TestcomputeRevoluteJacobianNum < matlab.unittest.TestCase
    % TestcomputeRevoluteJacobianNum performs unit testing for computeRevoluteJacobianNum
    
    properties
    end
    
    methods (Test)
        function testComputationNoRotation(testCase)
            % testComputationIdentity tests computation with identity rotation.
            ui = [-1 -1 -1]'; 
            pose = [1 0 0 1 0 0 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]';  
            velocity = [0 1 0 0 1 0]';
            actSol = computeRevoluteJacobianNum(ui, vi, vj, pose, 1e-6) * velocity;
            [fRevolute, expSol] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 70*sqrt(eps));
        end
        
        function testComputation90Rotation(testCase)
            % testComputation90Rotation tests computation 90 degrees rotation
            % around y axis.
            ui = [-1 -1 -1]'; 
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]';  
            velocity = [0 1 0 0 1 0]';
            actSol = computeRevoluteJacobianNum(ui, vi, vj, pose, 1e-6) * velocity;
            [fRevolute, expSol] = computeRevoluteJointConstraints(ui, vi, vj, pose, velocity);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 60*sqrt(eps));
        end
        
    end
end

