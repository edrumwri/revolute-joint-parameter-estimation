classdef TestcomputeSphericalJointConstraints < matlab.unittest.TestCase
    % TestcomputeSphericalJointConstraints performs unit testing for computeSphericalJointConstraints
    
    properties
    end

    methods (Test)   
        function testComputationIdentity(testCase)
            % testComputationIdentity no rotation just addition of two vectors.
            ui = [-1 0 0]';
            pose = [1 0 0 1 0 0 0]';
            actSol = computeSphericalJointConstraints(ui, pose);
            expSol = pose(1:3)+ eye(3) * ui;
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testComputation90(testCase)
            % testComputation90 tests computation with 90 degrees rotation around y.
            ui = [-1 0 0]';
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            actSol = computeSphericalJointConstraints(ui, pose);
            R = [0 0 1; 0 1 0; -1 0 0];
            expSol = pose(1:3) + R * ui;
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 4*sqrt(eps));
        end
    end
end
