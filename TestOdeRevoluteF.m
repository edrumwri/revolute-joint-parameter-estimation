classdef TestOdeRevoluteF < matlab.unittest.TestCase
    % TestOdeRevoluteF unit testing for odeRevoluteF
    
    properties
    end
    
    methods (Test)
        function sanityTest(testCase)
            % Returns expected values
            mass = 10;
            sphere_radius = 3;
            ui = [-1; 0; 0];
            vi = [0; 1; 0];
            vj = [0; 1; 0];

            % Moment of inertia tensor for a hallow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            init = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            
            actSol = odeRevoluteF(init, mass, J, ui, vi, vj);
            %not sure how to test this
            %expSol = 
            %testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end 
    end
end

