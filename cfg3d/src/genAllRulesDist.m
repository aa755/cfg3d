function [  ] = genAllRulesDist()
files = dir('rule_*');

num_files = length(files)

for i=1:num_files
    generateDistributionParameters(files(i).name)
end

end