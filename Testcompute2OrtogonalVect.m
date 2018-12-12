classdef Testcompute2OrtogonalVect < matlab.unittest.TestCase
    % Testcompute2OrtogonalVect: Unit testing for compute2OrtogonalVect.
    %   testComputation tests the computation against a know solution.
    %   testOrtogonal tests if the 3 vectors form an ortogonal triad.
    
    properties
    end

    methods (Test)   
        function testComputation(testCase)
            % testComputation tests against a know solution. 
            vi = [1 0 -3]';
            [vi1, vi2] = compute2OrtogonalVect(vi);
            actSol = [vi1, vi2];
            expSol = [[3 -3 1]', [-9 -10 -3]'];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testOrtogonal(testCase)
            % testOrtogonal tests to see if the 3 vectors are perpendicular. 
            vi = [1 0 -3]';
            [vi1, vi2] = compute2OrtogonalVect(vi);
            actSol = [vi1' * vi, vi2' * vi, vi1' * vi2];
            expSol = [0 0 0];
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
    end
end

