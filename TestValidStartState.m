classdef TestValidStartState < matlab.unittest.TestCase
    % TestValidStartState unit testing for validStartState.
    
    properties
    end
    
    methods (Test)
        %   ub is a 3x1 vector from the center-of-mass of the bob to the revolute 
        %       joint location and expressed in the bob frame. 
        %   vb is a 3x1 unit vector that points along the axis of the revolute joint 
        %       expressed in the bob frame. 
        %   vj is a 3x1 unit vector that points along the axis of the revolute joint 
        %       expressed in the j frame (fixed to the world).
        
        function testValid(testCase)
          % validStartState with good inital conditions.
          ub = [-1 0 0]';
          vb = [0 0 1]';
          vj = [0 0 1]';
          position = [1 0 0]';
          orientation = [1 0 0 0]';
          pose = [position; orientation];
          actSol = validStartState(pose, ub, vb, vj, 1e-16);
          expSol = 1;
          testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-16);
        end
        
        function testNOTValid(testCase)
          % validStartState with bad inital conditions.
          ub = [-1 0 0]';
          vb = [0 0 1]';
          vj = [0 0 1]';
          position = [0 0 0]';
          orientation = [1 0 0 0]';
          pose = [position; orientation];
          actSol = validStartState(pose, ub, vb, vj, 1e-16);
          expSol = 0;
          testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-16);
        end
    end
end

