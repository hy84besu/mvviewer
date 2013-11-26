function [shape,L,V,D] = demo(noEigs,scale,noScales,noSubscales)

    if nargin < 1
        noEigs = 20;
    end
    if nargin < 2
        scale=1;
    end
    if nargin < 3
        noScales=3;
    end
    
    if nargin < 4
        noSubscales=8;
    end

    file1= '../../datasets_matching/cedric/free/mesh_0101_0.off';
    file2= '../../datasets_matching/cedric/free/mesh_0115_0.off';
    
    %file1= '../../datasets_matching/antoine/mesh/mesh_130.off';
    %file2= '../../datasets_matching/antoine/outSorkFlow/mesh_130.off';

    shape = loadvectorfield(file1,file2);
    shape.F = sqrt(shape.DX.^2 + shape.DY.^2 + shape.DZ.^2);
    %shape.F = shape.DY;
    saveoff(shape,'../../test.off');
    
    shape = loadoff('../../datasets_matching/cedric/free_colour/mesh_0101.off');
    %shape = loadoff('../data/mesh_0120.off');
    %shape = loadoff('../data/mesh_0120_random.off');
    %shape = loadoff('../data/dino_sparse_1.off');
    saveoff(shape,'../../test.off');
    


    sigma = scale*avg_edge(shape);
    
    L = laplacian(shape);
    [V, D] = eigendecomposition(L,noEigs);
    scalespace(shape,V,D,sigma,noScales,noSubscales);
    
%     subplot(1,2,2);
%     shape=loadoff(file2);
%     trisurf(shape.TRIV,shape.X,shape.Y,shape.Z,shape.F), axis image, axis off, shading interp;%, colormap(gray);
%     view(-20,90);
    
end