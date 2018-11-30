clc
for i=1:size(a,1)-1
    disp(sprintf('"d_param_%d": %0.8f,',i, a(i,1)));
end
disp(sprintf('"d_param_%d": %0.8f',size(a,1), a(size(a,1),1)));