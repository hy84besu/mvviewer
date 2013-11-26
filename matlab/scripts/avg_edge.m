function e = avg_edge(shape)

IDX = [shape.TRIV(:,1) shape.TRIV(:,2); shape.TRIV(:,2) shape.TRIV(:,3); shape.TRIV(:,3) shape.TRIV(:,1)];

V = [shape.X shape.Y shape.Z];

V1 = V(IDX(:,1),:);
V2 = V(IDX(:,2),:);

EDiff = (V1 - V2)';
EDiffSq = EDiff.*EDiff;

e = mean (sqrt(sum(EDiffSq)));

end