function L = laplacian(shape )

N = size(shape.X,1);

F = shape.F;


IDX = [shape.TRIV(:,1) shape.TRIV(:,2); shape.TRIV(:,2) shape.TRIV(:,3); shape.TRIV(:,3) shape.TRIV(:,1)];
% flip the indices as well, to get around the holes
IDX = unique( [IDX; [ IDX(:,2) IDX(:,1)]],'rows');

% E - edge matrix
E = sparse( IDX(:,1), IDX(:,2), ones(size(IDX,1),1));

% scalar function difference
Gvals = abs(F(IDX(:,1)) - F(IDX(:,2)));
Gmean = mean(F);
Gstd = std(F);
GValsNorm = normpdf(Gvals,0,Gstd);
Gvals = GValsNorm;
% contributions from the scalar function
Gcontrib =  sparse( IDX(:,1), IDX(:,2), Gvals);

%vertex difference
V = [shape.X shape.Y shape.Z];
Vvals = abs(V(IDX(:,1),:) - V(IDX(:,2),:));
Vvals = sqrt(Vvals(:,1).^2 + Vvals(:,2).^2 + Vvals(:,3).^2);
Vmean = mean(Vvals);
Vstd = std(Vvals);
VValsNorm = normpdf(Vvals,0,Vstd);
Vvals = VValsNorm;
% contributions from shape (cotan weights, 1, etc)
Econtrib = sparse( IDX(:,1), IDX(:,2), Vvals);



W = (Gcontrib + E);
%W = E;

EYE = sparse( 1:1:N, 1:1:N, ones(N,1));

%degree along the diagonal
DEG = sparse( 1:1:N, 1:1:N, sum(W));

L = DEG - W;

%small = 0.5;
%Fv = shape.F + small;
%Fv = (Fv - min(Fv) + small) / (max(Fv) - min(Fv) + small);
%Fv = ones(N,1);
%FF = sparse( 1:1:N, 1:1:N,Fv);
%L = L * FF;

%FMat = sparse(E.*repmat(shape.F',N,1));
%FDiff = abs(FMat - FMat');
%L = L.*(FMat + 1);