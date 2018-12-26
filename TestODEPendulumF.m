classdef TestODEPendulumF < matlab.unittest.TestCase
    % TestODEPendulumF unit testing for odePendulumF
    
    properties
    end
    
    methods (Test)
        function testAtRest(testCase)
            % tests for the at rest case
            mass = 0;
            sphere_radius = 1;
            ui = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];

            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            init = [1; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            pose = init(1:7);
            if validStartState(pose, ub, vb, vj)
                actSol = odePendulumF(init, mass, J, ui, vb, vj);
                expSol = zeros(13,1);
                testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
            else
                error('TestODEPendulumF:testAtRest:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end
        end 
        
        function test90Rotation(testCase)
            % Tests for 90 degrees rotation
            mass = 1;
            sphere_radius = 1;
            ui = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];
            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            pose = [1 0 0 0.7071068 0 0.7071068 0]';
            velocity = [0 0 0 0 0 0]';
            init = [pose; velocity];
            if validStartState(pose, ub, vb, vj)
                dfdt = odePendulumF(init, mass, J, ui, vb, vj);
                vdot = dfdt(8:13);
                actSol = norm(vdot);
                expSol = 9.8;
                testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
            else
                error('TestODEPendulumF:test90Rotation:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end
        end 
    end
end

