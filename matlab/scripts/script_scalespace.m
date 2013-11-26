function [shape,L,V,D,Fscales,Gscales] = script_scalespace(meshIn, funcIn, fileOut, noEigs, noScales, noSubscales, sigma,display)

    if nargin < 1
        shape = loadoff('../data/mesh_0120.off');
        %shape = loadoff('../data/mesh_0120_random.off');
        %shape = loadoff('../data/dino_sparse_1.off');
    else
        shape = loadoff(meshIn);
    end

    if nargin < 2
    else if length(funcIn) > 2
        F = load(funcIn,'-ascii');
        shape.FOrig = shape.F;
        shape.F = F';
        %shape.F = normalize_scalar_function(F');
    end
    
    if nargin < 3
        fileOut = 'bla.txt';
    end

    if nargin < 4
        noEigs = 200;
    end
    
    if nargin < 5
        noScales=5;
    end

    if nargin < 6
        noSubscales=8;
    end

    if nargin < 7
        sigma = 2.0.^(1.0/(noSubscales-3));
        
        %avg_edge(shape);
    end
    if nargin < 8
        display=1;
    end
    
    %dims = bbox(shape);
    %avgDim = mean(dims(2,:) - dims(1,:));
    %sigma = log(avg_edge(shape))/log(2) / 2;
    
    %sigma = avgDim *baseSigmaRatio;
    %sigma = scale*avg_edge(shape);
    
    
    L = laplacian(shape);
    [V, D] = eigendecomposition(L,noEigs);
    [Fscales, Gscales] = scalespace(shape, V, D, sigma, noScales, noSubscales, display);
    
    %need to intertwine F and G and save them
    FGscales = zeros(size(Fscales,1)*2,size(Fscales,2));
    FGscales(1:2:(2*size(Fscales,1)-1),:) = Fscales;
    FGscales(2:2:(2*size(Fscales,1)  ),:) = Gscales;
    
    save(fileOut,'FGscales','-ascii');
end