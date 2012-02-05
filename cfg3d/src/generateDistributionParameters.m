function [  ] = generateDistributionParameters( featureFile )
%     M = magic(3);
%     dlmwrite('myfile.txt', [M*5 M/5], ' ')
% 
%     dlmwrite('myfile.txt', [M/3], '-append', ...
%        'roffset', 1, 'delimiter', ' ')

featureFile
    matrix = dlmread(featureFile);
    outputFileName = [featureFile '.model'];
    %delete(outputFileName);
    dlmwrite(outputFileName,'');
    [n,m] = size(matrix);
    matrix = matrix(:,1:m-1);
%    writeIndependentModels(outputFileName,matrix);
    writeModels(outputFileName,matrix)
   
end

function [] = writeIndependentModels(outputFileName,matrix)
    for i=1:m-1,
        [mean,sigma] = normfit(matrix(:,i));
        colMin = min(matrix(:,i));
        colMax = max(matrix(:,i));
        
        dlmwrite(outputFileName, [mean sigma colMin colMax], '-append');
    end
end

function [] = writeModels(outputFileName,matrix)
    meanv = mean(matrix);
    minv = min(matrix);
    maxv = max(matrix);
    covar=cov(matrix);
    numFeats=length(covar);
    rankm=rank(covar);
    fprintf(1,'%d,%d\n',rankm,numFeats);
    if(rankm<numFeats)
        covar=covar+0.001*eye(numFeats);
    end
    outmat=[covar;meanv;minv;maxv];   
    
    %colMin = min(matrix(:,i));
    %colMax = max(matrix(:,i));

    dlmwrite(outputFileName, outmat);
end
