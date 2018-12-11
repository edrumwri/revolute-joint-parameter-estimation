classdef TestSphericalJacobian < matlab.unittest.TestCase
    % TestSphericalJacobian unit tests for spherical jacobian.
    %   Two different poses are used to calculate the numerical and 
    %   analytical jacobian for the spherical joint and check the margin of
    %   error.
    
    properties
    end
    
    methods(Test)
        function noRotation(testCase)
            % noRotation compares the analitical and numerical with no
            % rotation
            position = [1 0 0]';
            quat = [1 0 0 0]';
            u = [1 0 0]';
            % degrees in radians change = 5.4542e-04
            eps = deg2rad(1/32); 
            actSol = computeAnalyticalJacobianSpherical(u, quat);
            expSol = computeNumJacobianSpherical(u, [position;quat], eps);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', eps);
        end
        function nintyRotation(testCase)
            % noRotation compares the analitical and numerical with 90
            % degrees rotation around y axes
            position = [1 0 0]';
            quat = [0.7071068 0 0.7071068 0]';
            u = [1 0 0]';
            % degrees in radians change = 5.4542e-04
            eps = deg2rad(1/32); 
            actSol = computeAnalyticalJacobianSpherical(u, quat);
            expSol = computeNumJacobianSpherical(u, [position;quat], eps);
            testCase.verifyEqual(actSol, expSol, 'AbsTol', eps);
        end
    end
end

