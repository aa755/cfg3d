function [  ] = generateDistributionParameters( featureFile )
%     M = magic(3);
%     dlmwrite('myfile.txt', [M*5 M/5], ' ')
% 
%     dlmwrite('myfile.txt', [M/3], '-append', ...
%        'roffset', 1, 'delimiter', ' ')

    matrix = dlmread(featureFile);
    outputFileName = [featureFile '.out'];
    dlmwrite(outputFileName,'');
    [n,m] = size(matrix);
    matrix = matrix(:,1:m-1);
    for i=1:m-1,
        [mean,sigma] = normfit(matrix(:,i));
        colMin = min(matrix(:,i));
        colMax = max(matrix(:,i));
        
        dlmwrite(outputFileName, [mean sigma colMin colMax], '-append');
    end
   
end

