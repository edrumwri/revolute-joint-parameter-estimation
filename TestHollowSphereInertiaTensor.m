classdef TestHollowSphereInertiaTensor < matlab.unittest.TestCase
    % TestHollowSphereInertiaTensor: Unit testing for
    % getHollowSphereInertiaTensor(mass, sphere_radius).
    %   This runs a sanity test: the hollow sphere inertia matrix has a
    %   closed form solution.
    
    properties
    end
    
    methods (Test)
        function sanityTest(testCase)
            % Since this is a closed form solution.
            mass = 10;
            sphere_radius = 1;
            actSol = getHollowSphereInertiaTensor(mass, sphere_radius);
            expSol = eye(3) * 2/3 * mass * sphere_radius^2;
            testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end 
    end
end

