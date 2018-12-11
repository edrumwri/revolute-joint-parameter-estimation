classdef TestcomputeNikraveshG < matlab.unittest.TestCase
    % TestcomputeNikraveshG performs unit testing for the Nikravesh G matrix. 
    %   First test verifies that each row of Nikravesh G is ortogonal to the
    %   quaternion used to calculate it.
    %   Second test verifies that the rows of Nikravesh G are ortogonal.
    
    properties
    end
    
    methods (Test)
        function testOrtogonality(testCase)
            % testOrtogonality verifies that each row of G is ortogonal to
            % quaternion.
            quat = [0.7071068; 0; 0.7071068; 0];
            actSol = computeNikraveshG(quat) * quat;
            expSol = zeros(3, 1);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end  
        
        function rowsOrtogonal(testCase)
            % rowsOfGOrtogonal tests if the rows of Nikravesh G are ortogonal.
            quat = [0.7071068; 0; 0.7071068; 0];
            NG = computeNikraveshG(quat);
            actSol = NG * NG';
            expSol = eye(3);
            % Absolute error 4 * sqrt(eps) = 5.9605e-08
            testCase.verifyEqual(actSol, expSol, 'AbsTol', 4 * sqrt(eps));
        end 
    end
end

