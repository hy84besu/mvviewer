function [t u] = loadfeat(filename)

f = fopen(filename, 'rt');

s = fgetl(f);
n = str2num(s);
t = [n(1) + 1];
u = [n(2:4)];

while ~isempty(s) & ischar(s)
    s = fgetl(f);
    if ischar(s)
        n = str2num(s);
        t = [t; n(1) + 1];
        u = [u; n(2:4)];
    end
end
    

fclose(f);
