function sum = gaussianProb( loc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    means=[0.26621 0.22 0.41074];
    sigmas=[0.015255 0.0248 0.010168];
  %  sigmas=[1 1 1];
    centroid{1}=[-0.215402 -0.35645 0.430626];
    centroid{2}=[-0.197128 -0.430894 0.256549];
    centroid{3}=[-0.0228314 -0.347311 0.259338];

    sum=0;
    for i=1:3
        temp=helper(loc,centroid{i},means(i),sigmas(i));
        sum=sum+temp;
    end

end

function p=helper(loc,centroid,mean,sigma)
    dist=(norm(loc-centroid))
    p=((dist-mean)^2)/(2*(sigma^2))
end