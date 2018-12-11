classdef TestHallowSphereInertiaTensor < matlab.unittest.TestCase
    %TestHallowSphereInertiaTensor Unit testing for getHallowSphereInertiaTensor
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Test)
        function dumTest(testCase)
            m = randi(10,1,1);
            r = randi(3,1,1);
            actSol = getHallowSphereInertiaTensor(m,r);
            expSol = diag([2/3*m*r^2, 2/3*m*r^2, 2/3*m*r^2]);
            testCase.verifyEqual(actSol,expSol,'AbsTol',sqrt(eps));
        end 
    end
end

