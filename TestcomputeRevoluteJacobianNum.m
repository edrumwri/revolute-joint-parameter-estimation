classdef TestcomputeRevoluteJacobianNum < matlab.unittest.TestCase
    % TestcomputeRevoluteJacobianNum performs unit testing for computeRevoluteJacobianNum
    
    properties
    end
    
    methods (Test)
        function testNumericalVsAnalyticalJac(testCase)
            % testNumericalVsAnalyticalJac compares the analytical jacobian
            % with the numerical one for the revolute joint.
            % This implicitly testes the spherical joint too as the
            % revolute joint is a spherical one with two additional
            % constraints. 
            
            ui = [-1 -1 -1]'; 
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            vi = [0 1 0]'; 
            vj = [0 1 0]';  
            actSol = computeRevoluteJacobianNum(ui, vi, vj, pose,  1e-6); 
            expSol = computeRevoluteJacobian(ui, vi, vj, pose(4:7));
            testCase.verifyEqual(actSol, expSol, 'AbsTol',1e-6);
        end
    end
end

