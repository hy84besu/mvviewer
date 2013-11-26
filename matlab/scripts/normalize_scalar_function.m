function F2 = normalize_scalar_function(F)


Fmean = mean(F);
Fstd = std(F);

N=3*Fstd; % how many std devs to consider


Fnorm = (F - Fmean)/N;

Fscaled = max(-1,min(Fnorm,1));

F2 = Fscaled;

