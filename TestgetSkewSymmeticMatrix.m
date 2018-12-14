classdef TestgetSkewSymmeticMatrix < matlab.unittest.TestCase
    % TestgetSkewSymmeticMatrix  performs unit testing for genetrating a 
    % 3x3 skew symmetric matrix using getSkewSymmeticMatrix function
    
    properties
    end
    
    methods (Test)
        function testSkewSymmetricBase(testCase)
           % testSkewSymmetricBase compares against a known solution 
           v = [1 2 3];
           actSol = getSkewSymmetricMatrix(v);
           expSol = [0    -3     2;
                     3     0    -1;
                    -2     1     0];
           testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
        
        function testSkewSymmetricGeneric(testCase)
           % testSkewSymmetricGeneric looks at the general case: 
           % a square matrix, A, is skew-symmetric if it is equal to the 
           % negation of its nonconjugate transpose, A = -A.'.
           v = rand(3,1);
           A = getSkewSymmetricMatrix(v);
           actSol = A;
           expSol = -A.';
           testCase.verifyEqual(actSol, expSol, 'AbsTol', sqrt(eps));
        end
     
    end
end

