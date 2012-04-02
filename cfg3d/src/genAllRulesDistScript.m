function [  ] = genAllRulesDistScript( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
for i=1:4
    cd(['rules','0'+i])
    genAllRulesDist
    cd ..
end

%exit

end

