function saveply(shape,filename)

f = fopen(filename, 'w');

fprintf(f,'ply\n');
fprintf(f,'format ascii 1.0\n');
fprintf(f,'comment Matlab generated\n');
fprintf(f,'element vertex %d\n',size(shape.X,1));
fprintf(f,'property float x\n');
fprintf(f,'property float y\n');
fprintf(f,'property float z\n');
fprintf(f,'property float quality\n');
fprintf(f,'element face %d\n',size(shape.TRIV,1));
fprintf(f,'property list uchar int vertex_indices\n');
fprintf(f,'end_header\n');

for x=1:size(shape.X,1)
    fprintf(f,'%f %f %f %f\n',shape.X(x),shape.Y(x), shape.Z(x), shape.F(x));
end

for x=1:size(shape.TRIV,1)
    fprintf(f,'3 %d %d %d\n',shape.TRIV(x,1)-1, shape.TRIV(x,2)-1, shape.TRIV(x,3)-1);
end

fclose(f);