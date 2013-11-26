% returns the radius of a circle that represents percentage of the total
% surface
function res=radius_area_surface(shape,percentage)

 area = area_surface(shape);
 res = sqrt((percentage/100.0)*area/(pi));
 

b = bbox(shape);
diagonal = norm(b(1,:)-b(2,:));
res = diagonal*(percentage/100.0);
