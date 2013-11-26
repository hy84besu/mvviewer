function F2 = normalized_gradient(F)


Fmean = mean(F);
Fstd = std(F);

N=3; % how many std devs to consider


Fnorm = (F - Fmean)/N;

Fscaled = max(-1,min(Fnorm,1));

F2 = Fscaled;

