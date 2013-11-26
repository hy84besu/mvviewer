function [Fscales, Gscales] = scalespace(shape, V, D, sigma, noScales, noSubscales, display)


if nargin < 5
    noScales = 3;
end

if nargin < 5
    noSubscales = 7;
end


if nargin < 7
    display = 1;
end


F = shape.F;
N = size(F,1);

show_orig=0;

%original eigenvectors
lambda = (diag(D));

%projections on F
beta = (F'*V)';

%init first F
t = 0;
Ft = sum((V * diag(beta.* exp(-t*lambda)))')';
FPrev = Ft;

%init results
Fscales = zeros(noScales*noSubscales,N);
Gscales = Fscales;

dims = bbox(shape);
avgDim = mean(dims(2,:) - dims(1,:));
avgEdge = avg_edge(shape);
mult = log(avgDim/avgEdge)/log(2) / 2;
    
edge = 20*mult*avgEdge;
edge = avgEdge;
edge = radius_area_surface(shape,1)/avgEdge;
%edge = 1;

sigma=1;
k=1;

t0 = (edge.^2)/2;

if display==1
figure(1);
clf(1);
colormap(jet);
end

for i=1:(noScales)
    %sigmaCur=2.^(i-1)*sigma;
    %sigmaScale = ((2).^(i-1));
    %sigmaScale  = 2.0.^(1.0/(noSubscales));
    for j=1:(noSubscales)
        sigmaCur = (2.^((i-1) + (j-1)/(noSubscales)));
        
        fprintf('scale = %02d subscale = %02d sigma=%2.2f\n',i,j,sigmaCur);

        %t = ((edge*sigmaCur).^2) / 2;
        t = t0*sigmaCur;
        FCur = sum((V * diag(beta.* exp(-t*lambda)))')';
        GCur = FCur - FPrev;
        if (i==1) & (j==1)
            FCur = F;
        end
        Fscales(k,:) = FCur;
        Gscales(k,:) = GCur;

        % advance 
        FPrev = FCur;
        %sigmaCur = sqrt( sigmaCur*sigmaCur + sigmaScale*sigmaScale);        
        k=k+1;
        
        %sigmaCur = sigmaCur * sqrt(2);
        if display == 1
            %viewCoords = [-180 -60];
            viewCoords = [-1 -90];
            f = figure(1);
            if show_orig == 1
                subplot(1,2,1);
            end
            trisurf(shape.TRIV,shape.X,shape.Y,shape.Z,FCur), axis image, axis off, shading interp;%, colormap(gray);
            view(viewCoords);
            drawnow

            if show_orig == 1
                colorbar;                
                subplot(1,2,2)
                trisurf(shape.TRIV,shape.X,shape.Y,shape.Z,F), axis image, axis off, shading interp;%, colormap(gray);
                view(viewCoords);
                drawnow
                
                colorbar;
            end
        
            %colormap(jet);
            if (j==1)
                filename = sprintf('images/scalespace/scale-%d.png',i);
                saveas(f,filename);
            end
            
        end    
    end
end

% figure(2);
% 
% subplot(1,2,1);
% hist(Ft,20);
% 
% subplot(1,2,2)
% hist(F,20);

%drawnow

