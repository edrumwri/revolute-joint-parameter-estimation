classdef TestcomputeSphericalJointConstraints < matlab.unittest.TestCase
    % TestcomputeSphericalJointConstraints performs unit testing for computeSphericalJointConstraints
    
    properties
    end

    methods (Test)   
        function testComputationIdentity(testCase)
            % testComputationIdentity tests computation with identity rotation.
            ui = [-1 0 0]';
            pose = [1 0 0 1 0 0 0]';
            actSol = computeSphericalJointConstraints(ui, pose);
            expSol = pose(1:3) + eye(3) * ui;
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
        
        function testNoVelocityInputFDot(testCase)
           % testNoVelocityInputFDot tests if the function returns zeros 
            % if no velocity for fDot.
            ui = [-1 0 0]';
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            [fSpherical, actSol] = computeSphericalJointConstraints(ui, pose);
            expSol = zeros(3,1);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testNoAccelerationInputFDDot(testCase)
           % testNoAccelerationInputFDDot tests if the function returns zeros 
            % if no acceleration for fDDot.
            ui = [-1 0 0]';
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            [fSpherical, fDSpherical, actSol] = computeSphericalJointConstraints(ui, pose);
            expSol = zeros(3,1);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
    end   
end

