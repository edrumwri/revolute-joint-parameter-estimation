classdef TestODEPendulumF < matlab.unittest.TestCase
    % TestODEPendulumF unit testing for odePendulumF
    
    properties
    end
    
    methods (Test)
        % For all these methods the system of coordinates is: 
        % +x to the right, +y up, +z out of the page.
        %   ub is a 3x1 vector from the center-of-mass of the bob to the revolute 
        %       joint location and expressed in the bob frame. 
        %   vb is a 3x1 unit vector that points along the axis of the revolute joint 
        %       expressed in the bob frame. 
        %   vj is a 3x1 unit vector that points along the axis of the revolute joint 
        %       expressed in the j frame (fixed to the world).
        
        function testPendulumFromHorizontal(testCase)
            % The pendulum is horizontal (the bob lies on +x). 
            mass = 1;
            sphere_radius = 0;
            ub = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];
            position = [1 0 0]';
            orientation = [1 0 0 0]';
            pose = [position; orientation];
            linearVel = zeros(3,1);
            angularVel = [0 0 0]'; 
            velocity = [linearVel; angularVel];
            init = [pose; velocity];
            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            if validStartState(pose, ub, vb, vj)
                dfdt = odePendulumF(init, mass, J, ub, vb, vj);
                % Expected  
                % linearAcc = [0 -9.81 0]';
                angularAcc = [0 0 -9.81]';
                actSol = norm(dfdt(11:13));
                expSol = norm(angularAcc);
                testCase.verifyEqual(actSol, expSol, 'AbsTol', 1e-14);
            else
                error('TestODEPendulumF:testAtRest:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end       
        end
        
        function testPendulumAtRest(testCase)
            % The pendulum is vertical (0 with the vertical axis). 
            mass = 1;
            sphere_radius = 0;
            ub = [-1; 0; 0];
            vb = [0; 0; 1];
            vj = [0; 0; 1];
            position = [0 -1 0]';
            orientation =  qt([0 0 1], -pi/2);
            pose = [position; orientation];
            velocity = zeros(6,1);
            init = [pose; velocity];
            % Moment of inertia tensor for a hollow sphere of radius r and mass m
            J = getHollowSphereInertiaTensor(mass, sphere_radius);
            if validStartState(pose, ub, vb, vj, eps) % Matlab defines eps = 2.220446049250313e-16
                dfdt = odePendulumF(init, mass, J, ub, vb, vj);
                % Expected  
                % linearAcc = [0 0 0]';
                % angularAcc = [0 0 0]';
                actSol = dfdt;
                expSol = zeros(13,1);
                testCase.verifyEqual(actSol, expSol, 'AbsTol', 2.55e-15);
            else
                error('TestODEPendulumF:testAtRest:ConstraintsNotMeet','Initial conditions are not valid.'); 
            end       
        end
        
    end
end

