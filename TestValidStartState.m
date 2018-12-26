classdef TestValidStartState < matlab.unittest.TestCase
    % TestValidStartState unit testing for validStartState.
    
    properties
    end
    
    methods (Test)
        function testValid(testCase)
          % validStartState with good inital conditions.
          pose = [1 0 0 1 0 0 0]';
          ub = [-1; 0; 0];
          vb = [0; 0; 1];
          vj = [0; 0; 1];
          actSol = validStartState(pose, ub, vb, vj);
          expSol = 1;
          testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testNOTValid(testCase)
          % validStartState with bad inital conditions.
          pose = [0 0 0 1 0 0 0]';
          ub = [-1; 0; 0];
          vb = [0; 0; 1];
          vj = [0; 0; 1];
          actSol = validStartState(pose, ub, vb, vj);
          expSol = 0;
          testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
    end
end

