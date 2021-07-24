function [s] = Sampler(a,b,N,M)

vec = a + (b-a).*rand(N,M);

s = vec.';

end


