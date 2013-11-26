function shape = loadvectorfield(filename1,filename2)

shape = loadoff(filename1);
shape2 = loadoff(filename2);

shape.DX = shape2.X - shape.X;
shape.DY = shape2.Y - shape.Y;
shape.DZ = shape2.Z - shape.Z;