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
    matrix = matrix(:,1:m-1); % terminating coomans in each line add additional column
%    writeIndependentModels(outputFileName,matrix);
    writeMoGs(outputFileName,matrix)
   
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
    %if(rankm<numFeats)
    covar=covar+eye(numFeats);
    %end
    assert(rank(covar)==numFeats);
    sigmaInv=inv(covar);
    const=(-log(det(sigmaInv)));
    constV=ones(1,numFeats)*const;
    outmat=[meanv;minv;maxv;sigmaInv;constV];   
    (-log(det(inv(covar))))
    assert((-log(det(inv(covar))))>=0);
    
    %colMin = min(matrix(:,i));
    %colMax = max(matrix(:,i));

    dlmwrite(outputFileName, outmat);
end

function [] = writeMoGs(outputFileName,matrix)
    meanv = mean(matrix);
    minv = min(matrix);
    maxv = max(matrix);
    covar=cov(matrix);
    numFeats=length(covar);
    rankm=rank(covar);
    fprintf(1,'%d,%d\n',rankm,numFeats);
    if(rankm<numFeats)
        %if(rankm<numFeats)
        covar=covar+eye(numFeats);
        %end
        assert(rank(covar)==numFeats);
        sigmaInv=inv(covar);
        const=(-log(det(sigmaInv)));
        constV=ones(1,numFeats)*const;
        outmat=[minv;maxv;sigmaInv;constV;meanv;ones(1,numFeats)];   
        (-log(det(inv(covar))))
        assert((-log(det(inv(covar))))>=0);
    else
        
        
        bestBIC=inf;
        try
            for i=1:3
%                model=gmdistribution.fit(matrix,i,'SharedCov',true,'CovType','diagonal');
                model=gmdistribution.fit(matrix,i,'SharedCov',true);
                if(bestBIC>model.BIC)
                    bestBIC=model.BIC
                    bestModel=model;
                end
            end
        catch err
            assert(i>1);
        end
%        covar=diag(bestModel.Sigma);
        covar=bestModel.Sigma;
        %if(rankm<numFeats)
        covar=covar+eye(numFeats);
        %end
        assert(rank(covar)==numFeats);
        assert((-log(det(inv(covar))))>=0);
        sigmaInv=inv(covar);
        const=(-log(det(sigmaInv)));
        constV=ones(1,numFeats)*const;
        outmat=[minv;maxv;sigmaInv;constV];
        (-log(det(inv(covar))))
        bestModel.NComponents
        for i=1:bestModel.NComponents
            mixPV=ones(1,numFeats)*bestModel.PComponents(i);
            outmat=[outmat ; bestModel.mu(i,:); mixPV];
        end
        
        
    end    

    dlmwrite(outputFileName, outmat);
end
