function B = bbox(shape)


V = [shape.X shape.Y shape.Z];

Vmin = min(V);
Vmax = max(V);

B = [Vmin; Vmax];
end