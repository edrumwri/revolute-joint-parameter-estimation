classdef TestcomputeNikraveshG < matlab.unittest.TestCase
    % TestcomputeNikraveshG performs unit testing for the Nikravesh G matrix. 
    %   testOrtogonality verifies that each row of Nikravesh G is ortogonal to the
    %       quaternion used to calculate it.
    %   rowsOfGOrthogonal verifies that the rows of Nikravesh G are ortogonal.
    
    properties
    end
    
    methods (Test)
        function testOrthogonality(testCase)
            % testOrtogonality verifies that each row of G is orthogonal to
            % quaternion.
            quat = [0.7071068; 0; 0.7071068; 0];
            actSol = computeNikraveshG(quat) * quat;
            expSol = zeros(3, 1);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end  
        
        function rowsOrthogonal(testCase)
            % rowsOfGOrthogonal tests if the rows of Nikravesh G are orthogonal.
            quat = [0.7071068; 0; 0.7071068; 0];
            NG = computeNikraveshG(quat);
            actSol = NG * NG';
            expSol = eye(3);
            % Absolute error 4 * sqrt(eps) = 5.9605e-08
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 4 * sqrt(eps));
        end 
    end
end

