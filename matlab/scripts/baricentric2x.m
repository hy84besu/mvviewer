function X = baricentric2x(tx,ux,shape)

ux = [ux zeros(size(ux,1),3-size(ux,2))];
ux(:,3) = 1 - sum(ux(:,1:2),2);

XX = [shape.X shape.Y shape.Z];
TT = shape.TRIV(tx,:);


X = XX(TT(:,1),:).*repmat(ux(:,1),[1 3]) + XX(TT(:,2),:).*repmat(ux(:,2),[1 3]) + XX(TT(:,3),:).*repmat(ux(:,3),[1 3]);


