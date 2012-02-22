function [ ] = aggregatePR( names )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
len=length(names);
sump=0;
sumr=0;
sumc=0;
%!git log | head
%!git diff > tempGit
%!cat tempGit
precSum=0;
recSum=0;
lenp=0;
lenr=0;

    fprintf(1,'name \t\t prec \t recal \t count\n');
for i=1:len
    M=csvread(strcat(names{i},'.out'));
    prec{i}=sum(M(:,3))/sum(M(:,2));
    sump=sump+sum(M(:,2));
    recall{i}=sum(M(:,3))/sum(M(:,1));
    sumr=sumr+sum(M(:,1));
    correct{i}=sum(M(:,3));
    sumc=sumc+correct{i};
    fprintf(1,'%10s \t %4.2f \t%4.2f \t%4.2f \n',names{i} ,prec{i},recall{i},correct{i});
    if(sum(M(:,2))~=0)
        precSum=precSum+prec{i};
        lenp=lenp+1;
    end
    if(sum(M(:,1))~=0)
        recSum=recSum+recall{i};
        lenr=lenr+1;
    end
end
    fprintf(1,'%10s \t %4.2f \t%4.2f \t%4.2f \n','micro' ,sumc/sump,sumc/sumr,sumc);
    fprintf(1,'%10s \t %4.2f \t%4.2f \t%4.2f \n','macro' ,precSum/lenp,recSum/lenr,sumc);

end

