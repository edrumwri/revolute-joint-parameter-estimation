classdef TestcomputeRevoluteJointConstraints < matlab.unittest.TestCase
    % TestcomputeRevoluteJointConstraints performs unit testing for computeRevoluteJointConstraints
    
    properties
    end

    methods (Test)   
        function testComputationIdentity(testCase)
            % testComputationIdentity tests computation with identity rotation.
            ui = [-1 0 0]';
            pose = [1 0 0 1 0 0 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]'; 
            actSol = computeRevoluteJointConstraints(ui, vi, vj, pose);
            expSol = [pose(1:3) + eye(3) * ui; 0 ; 0];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testComputation90(testCase)
            % testComputation90 tests computation with 90 degrees rotation around y.
            ui = [-1 0 0]';
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]'; 
            actSol = computeRevoluteJointConstraints(ui, vi, vj, pose);
            R = [0 0 1; 0 1 0; -1 0 0];
            expSol = [pose(1:3) + R * ui; 0; 0];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 4*sqrt(eps));
        end
        
        function testNoVelocityInputFDot(testCase)
           % testNoVelocityInputFDot tests if the function returns zeros 
            %if no velocity for fDot.
            ui = [-1 0 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]'; 
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            [fRev,actSol] = computeRevoluteJointConstraints(ui, vi, vj, pose);
            expSol = zeros(5,1);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
    end
end
