function saveoff(shape,filename)

f = fopen(filename, 'w');

fprintf(f,'COFF\n');
fprintf(f,'%d %d 0\n',size(shape.X,1),size(shape.TRIV,1));
for x=1:size(shape.X,1)
    fprintf(f,'%f %f %f ',shape.X(x),shape.Y(x), shape.Z(x));
    for i=1:3 
        fprintf(f,'%f ',shape.F(x));
    end
    fprintf(f,'1\n');
end

for x=1:size(shape.TRIV,1)
    fprintf(f,'3 %d %d %d\n',shape.TRIV(x,1)-1, shape.TRIV(x,2)-1, shape.TRIV(x,3)-1);
end

fclose(f);