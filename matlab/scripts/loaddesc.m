function [desc] = loaddesc(filename)

f = fopen(filename, 'rt');

s = fgetl(f);
desc = str2num(s);

while ~isempty(s) & ischar(s)
    s = fgetl(f);
    if ischar(s)
        desc = [desc; str2num(s)];
    end
end
    

fclose(f);
