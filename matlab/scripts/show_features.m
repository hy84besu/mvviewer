function show_features(shape, t,u,desc, scale,v)

K = size(desc,2);
%cmap = hsv(max(N,1)); 

C = [desc(:,1:min(K,3)) zeros(length(t),3-K)];
C = C - min(C(:));
C = C / max(C(:));

I = zeros(length(t),1,3);
I(:,1,:) = C;
[c,cmap] = rgb2ind(I,length(t));
N = size(cmap,1);
c = double(c);

cmap=colormap(gray);
c = ones(size(c));

XX  = baricentric2x(t,u,shape);

[Xs,Ys,Zs] = sphere(40);

clf;
hold on
trisurf(shape.TRIV,shape.X,shape.Y,shape.Z,N+1,'SpecularExponent',100);



for m = 1:length(t)
    surf(Xs*scale+XX(m,1),Ys*scale+XX(m,2),Zs*scale+XX(m,3),'CData',repmat(c(m)+1,size(Xs)));
end

axis image, axis off, shading flat
view(v)
camlight head
lighting phong

colormap([cmap; 1 0.95 0.9]);
caxis([1 N+1]);

