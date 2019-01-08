classdef TestODEPendulumF < matlab.unittest.TestCase
    % TestODEPendulumF unit testing for odePendulumF
    
    properties
    end
    
    methods (Test)
        function testAtRest(testCase)
            % tests for the at rest case
            mass = 1;
            sphere_radius = 0;
            ub = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];

            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            init = [1 0 0 1 0 0 0 0 0 0 0 0 0]';
            pose = init(1:7);
            if validStartState(pose, ub, vb, vj)
                dfdt = odePendulumF(init, mass, J, ub, vb, vj);
                actSol = dfdt;
                expSol = zeros(13,1);
                testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-15);
            else
                error('TestODEPendulumF:testAtRest:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end
        end 
        
        function test90Rotation(testCase)
            % Tests for 90 degrees rotation
            mass = 1;
            sphere_radius = 0;
            ub = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];
            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            % pi/2 rotation around y axis
            quat = qt([0 0 1], pi/2);
            position = [0 1 0]';
            pose = [position; quat];
            linearVel = zeros(3,1);
            angularVel = [0 0 1]';
            velocity = [linearVel; angularVel];
            init = [pose; velocity];
            if validStartState(pose, ub, vb, vj, eps) % Matlab defines eps = 2.220446049250313e-16
                dfdt = odePendulumF(init, mass, J, ub, vb, vj); 
                %quatDot = computeQuatDot(quat, angularVel);
                linearVelDot = [-9.81 0 0]';
                angularVelDot = [0 0 -9.81]';
                velocityDot = [linearVelDot; angularVelDot];
                vdot = dfdt(8:13);
                actSol = norm(vdot);
                expSol = norm(velocityDot); %[linearVel; quatDot; velocityDot];
                testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-16);
            else
                error('TestODEPendulumF:test90Rotation:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end
        end 
        
        function testNeg90Rotation(testCase)
            % Tests for -90 degrees rotation
            mass = 1;
            sphere_radius = 0;
            ub = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];
            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            % pi/2 rotation around y axis
            quat = qt([0 0 1], -pi/2);
            position = [0 -1 0]';
            pose = [position; quat];
            linearVel = zeros(3,1);
            angularVel = [0 0 -1]';
            velocity = [linearVel; angularVel];
            init = [pose; velocity];
            if validStartState(pose, ub, vb, vj, eps) % Matlab defines eps = 2.220446049250313e-16
                dfdt = odePendulumF(init, mass, J, ub, vb, vj);
                %quatDot = computeQuatDot(quat, angularVel);
                linearVelDot = [9.81 0 0]';
                angularVelDot = [0 0 9.81]';
                velocityDot = [linearVelDot; angularVelDot];
                vdot = dfdt(8:13);
                actSol = norm(vdot);
                expSol = norm(velocityDot); %[linearVel; quatDot; velocityDot];
                testCase.verifyEqual(actSol, expSol, 'AbsTol',1e-16);
            else
                error('TestODEPendulumF:test90Rotation:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end
        end 
    end
end

