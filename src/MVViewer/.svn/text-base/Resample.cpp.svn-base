/*
 *  Resample.cpp
 *  3D_Rec
 *
 *  Created by Andrei Zaharescu on 13/06/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "Resample.h"


/////////////////////////////////////////////////////////////////////////////////
// Reechantillonnage d'une image
// Gere le sous-echantillonnage et le sur-echantillonnage
// Version optimisee quand on divise l'image par un facteur entier
void Resample(const CImg<> &in, CImg<> &out) {
	
    // On lisse selon x et y en fonction de la reduction a appliquer
    CImg<> tmp(in);
    const int nx = in.width / out.width;
    const int ny = in.height / out.height;
    const float sx = float(in.width) / float(out.width);
    const float sy = float(in.height) / float(out.height);
    if (sx>1) tmp.deriche(factor*sx,0,'x');
    if (sy>1) tmp.deriche(factor*sy,0,'y');
	
    // On separe les cas 2D et 3D
    if (in.depth == 1) {
        // Si on divise par un facteur entier, pas de besoin d'interpolation lineaire
        if ((in.width%out.width==0) && (in.height%out.height==0))
            cimg_forXYV(out,x,y,k) out(x,y,0,k) = tmp(nx*x,ny*y,0,k);
        else
            cimg_forXYV(out,x,y,k) out(x,y,0,k) = float(tmp.linear_pix2d(sx*x,sy*y,0,k));
    }
    else {
        // On lisse selon z en fonction de la reduction a appliquer
        const int nz = in.depth / out.depth;
        const float sz = float(in.depth) / float(out.depth);
        if (sz>1) tmp.deriche(factor*sz,0,'z');
        
        // Si on divise par un facteur entier, pas de besoin d'interpolation lineaire
        if ((in.width%out.width==0) && (in.height%out.height==0) && (in.depth%out.depth==0))
            cimg_forXYZV(out,x,y,z,k) out(x,y,z,k) = tmp(nx*x,ny*y,nz*z,k);
        else
            cimg_forXYZV(out,x,y,z,k) out(x,y,z,k) = float(tmp.linear_pix3d(sx*x,sy*y,sz*z,k));
    }
}


CImg<> Resample(const CImg<> &in, int width, int height, int depth) {
    CImg<> out(width,height,depth,in.dim);
    Resample(in,out);
    return out;
}
