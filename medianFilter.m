function m = medianFilter(p_noisyRange,L,M)
% This function works as a median filter of order M
i = 1; p_filtered = zeros(L,1);
for k = M+1:L-M
    p_filtered(i) = median(p_noisyRange(k-M+1:k));  
    i=i+1;
end
m = p_filtered;
end