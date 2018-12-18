classdef TestcomputeRevoluteJacobianDotNum  < matlab.unittest.TestCase
    % TestcomputeRevoluteJacobianDotNum performs unit testing on
    % function computeRevoluteJacobianDotNum
    
   properties
    end

    methods (Test)  
        function testComputation90Rotation(testCase)
            % testComputation90Rotation tests computation 90 degrees rotation
            % around y axis.
            ui = [-1 -1 -1]'; 
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]';  
            velocity = [1 2 3 4 5 6]';
            acceleration = [2 3 4 5 6 7]';
            quatDot = computeQuatDot(pose(4:7), velocity(4:6));
            actSol = computeRevoluteJacobianDotNum(ui, vi, vj, pose, velocity, acceleration, 1e-6);
            [fRevolute, fDRevolute, expSol] = computeRevoluteJacobianDot(ui, vi, vj, pose(4:7), quatDot) * velocity;
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-6);
        end
    end
end

