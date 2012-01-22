function [ ] = aggregatePR( names )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
len=length(names);
sump=0;
sumr=0;
sumc=0;
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
    
    
end
    fprintf(1,'%10s \t %4.2f \t%4.2f \t%4.2f \n','macro' ,sumc/sump,sumc/sumr,sumc);

end

