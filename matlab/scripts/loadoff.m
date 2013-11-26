function shape = loadoff(filename)

shape = [];

f = fopen(filename, 'rt');

n = '';
while isempty(n)
    fgetl(f);
    n = sscanf(fgetl(f), '%d %d %d');
end
    
nv = n(1);
nt = n(2);
data = fscanf(f, '%f');

shape.TRIV = reshape(data(end-4*nt+1:end), [4 nt])';
shape.TRIV = shape.TRIV(1:nt,2:4) + 1;


data = data(1:end-4*nt);
data = reshape(data, [length(data)/nv nv]);

shape.X = data(1,:)';
shape.Y = data(2,:)';
shape.Z = data(3,:)';

shape.V = [shape.X shape.Y shape.Z];

if (size(data,1)>=6)
    shape.R = data(4,:)';
    shape.G = data(5,:)';
    shape.B = data(6,:)';

    shape.F = (shape.R + shape.G + shape.B) /3 ;
else
    shape.F = ones(size(data(1,:)));
end

fclose(f);