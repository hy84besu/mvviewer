function test(m)

% load shapes

if nargin <1
    basefile = 'shape1';
    basefile = '../../out_SHREC-2011/corr-det2-feat2-pt400/0001.rasterize.1';
    basefile = '../../results.txt';
else
    basefile = m;%sprintf('../../out_SHREC-2011/corr-det2-feat-2/0001.%s',m)';
end

off_file = sprintf('%s.off',basefile);
feat_file = sprintf('%s.feat',basefile);
desc_file = sprintf('%s.desc',basefile);

shape = loadoff(off_file);

% load feature points
[t u] = loadfeat(feat_file);

% load feature descriptors
desc = loaddesc(desc_file);

% show feature points + descriptors (color represents the first 3
% components of the descriptor)
%show_features(shape, t,u, desc, 0.02,[-30 15]);
%show_features(shape, t,u, desc, 0.03,[148 -76]);
%show_features(shape, t,u, desc, 3,[-95 -70]);
show_features(shape, t,u, desc, 5,[-1 -90]);
%show_features(shape, t,u, desc, 5,[-180 -60]);


