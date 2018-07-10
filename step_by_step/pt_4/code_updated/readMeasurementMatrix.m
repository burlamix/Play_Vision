function [M, MC] = readMeasurementMatrix( )
    M = textread('model house/measurement_matrix.txt');
    
    % Center it
    numPts = size(M,2);
    MC = M - repmat( sum(M,2) / numPts, 1, numPts);
   
end