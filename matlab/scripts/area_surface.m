function res=area_surface(shape)


A = cross(shape.V(shape.TRIV(:,2),:)-shape.V(shape.TRIV(:,1),:),shape.V(shape.TRIV(:,3),:) - shape.V(shape.TRIV(:,1),:));

Anorm = sqrt(A(:,1).^2 + A(:,2).^2 + A(:,3).^2);

res = sum(Anorm) / 2;

