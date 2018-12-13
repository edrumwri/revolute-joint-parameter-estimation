classdef TestcomputeBasisFromAxis < matlab.unittest.TestCase
    % TestcomputeBasisFromAxis performs unit testing for computeBasisFromAxis.
    %   testComputation tests the computation against a known solution.
    %   testOrthogonal tests if the 3 vectors form an orthogonal triad.
    %   testZerosInput tests if the error is thrown for a no axis vector (zeros(3,1))
    
    properties
    end

    methods (Test)   
        function testComputation(testCase)
            % testComputation tests against a known solution. 
            vi = [1 0 -3]';
            [vi1, vi2] = computeBasisFromAxis(vi);
            actSol = [vi1, vi2];
            expSol = [[3 -3 1]', [-9 -10 -3]'];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testOrthogonal(testCase)
            % testOrthogonal tests to see if the 3 vectors are perpendicular. 
            vi = [1 0 -3]';
            [vi1, vi2] = computeBasisFromAxis(vi);
            actSol = [vi1' * vi, vi2' * vi, vi1' * vi2];
            expSol = [0 0 0];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        function testZerosInput(testCase)
            testCase.verifyError(@()computeBasisFromAxis(zeros(3,1)),'computeBasisFromAxis:InputIsZeros');
        end
    end
end

