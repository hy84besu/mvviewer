#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string>
#include "Matrix3x4.h"
#include "Rigide.h"

using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Classe representant une camera perspective
class Camera : public Matrix3x4 {
public:
	// Parametres extrinseques
	Rigide rigide;

	// Parametres intrinseques
	float alphau, alphav, deltau, deltav;
	
	float scale;

	int width,height;
	int id;
	
	// Destructeur
	virtual ~Camera() {}


	// Constructeur par defaut
	Camera() {
		Identity();
	}


	// Constructeur par recopie
	Camera& operator = (const Camera &a) {
		Matrix3x4::operator = (a);
		rigide = a.rigide;
		alphau = a.alphau;
		alphav = a.alphav;
		deltau = a.deltau;
		deltav = a.deltav;
		scale = a.scale;
		width=a.width;
		height=a.height;
		return *this;
	}

	Camera& operator = (const Matrix3x4 &a) {
		Matrix3x4::operator = (a);
		UpdateParameters();
		return *this;
	}


	// Matrice identite
	void Identity() {
		Matrix3x4::Identity();
		UpdateParameters();
	}

	// Parametres intrinsec
	Matrix3x3 Intrinsic() {
		Matrix3x3 tmp;
		tmp(1,1) = alphau;
		tmp(2,2) = alphav;
		tmp(1,3) = deltau;
		tmp(2,3) = deltav;
		tmp(3,3) = 1;
		return tmp;
	}

	Vector3 IntrinsicPixelDirection(int p_x, int p_y){
		Vector3 x,X;
		x(1) = p_x;
		x(2) = p_y;
		x(3) = 1;
		Matrix3x3 K = Intrinsic();
		X = K.Inverse()*x;
		return X;
	}

	Vector3 PixelDirection(int p_x, int p_y){
		Vector3 x,X;
		x(1) = p_x;
		x(2) = p_y;
		x(3) = 1;
		Matrix3x3 K = Intrinsic();
		X = K.Inverse()*x;
		X = rigide.matrix.Transpose()*(X - rigide.vector);
		return X;
	}
	
	// Centre de la camera
	Vector3 Center() const {
		return - rigide.matrix.Transpose() * rigide.vector;
	}


	// Entrees sorties
	// Apres avoir lu la matrice il faut en deduire les parametres
	virtual int Read(std::istream &in) {
		int ret = Matrix3x4::Read(in);
		if (ret) return ret;
		scale = matrix.GetRow(3).Norm();		
		UpdateParameters();
		return 0;
	}


	// Mise a jour des parametres a partir de la matrice
	// Formula taken from: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node3.html
	void UpdateParameters() {
		const float n = matrix.GetRow(3).Norm();
		Vector3 q1 = matrix.GetRow(1) / n;
		Vector3 q2 = matrix.GetRow(2) / n;
		Vector3 q3 = matrix.GetRow(3) / n;
		Vector3 r = vector / n;

		Vector3 q13 = q1 ^ q3;
		Vector3 q23 = q2 ^ q3;
		alphau = q13.Norm();
		alphav = q23.Norm();
		deltau = q1 * q3;
		deltav = q2 * q3;
		rigide.matrix.SetRow(1, q3 ^ q13 / alphau);
		rigide.matrix.SetRow(2, q3 ^ q23 / alphav);
		rigide.matrix.SetRow(3, q3);

//		double tz = (r(3)>0?r(3):-r(3));
		double tz = r(3);
		rigide.vector(1) = ( r(1) - deltau * tz ) / alphau;
		rigide.vector(2) = ( r(2) - deltav * tz ) / alphav;
		rigide.vector(3) = tz;
	}


	// Mise a jour de la matrice a partir des parametres
	void UpdateMatrix() {
		Vector3 r1 = rigide.matrix.GetRow(1);
		Vector3 r2 = rigide.matrix.GetRow(2);
		Vector3 r3 = rigide.matrix.GetRow(3);
		matrix.SetRow(1, r1 * alphau + r3 * deltau);
		matrix.SetRow(2, r2 * alphav + r3 * deltav);
		matrix.SetRow(3, r3);
		vector(1) = alphau * rigide.vector(1) + deltau * rigide.vector(3);
		vector(2) = alphav * rigide.vector(2) + deltav * rigide.vector(3);
		vector(3) = rigide.vector(3);
	}


    // Application d'un facteur d'echelle a l'espace ou a l'image
    void ScaleWorld(float f) {
        ScaleWorld(f,f,f);
    }
    void ScaleWorld(float fx, float fy, float fz) {
        matrix.SetColumn(1, matrix.GetColumn(1) * fx);
        matrix.SetColumn(2, matrix.GetColumn(2) * fy);
        matrix.SetColumn(3, matrix.GetColumn(3) * fz);
		width=lroundf(width*fx);
		height=lroundf(height*fy);		
        UpdateParameters();
    }
    void ScaleImage(float f) {
        ScaleImage(f,f);
    }
    void ScaleImage(float fx, float fy) {
        matrix.SetRow(1, matrix.GetRow(1) * fx);
        matrix.SetRow(2, matrix.GetRow(2) * fy);
        vector(1) *= fx;
        vector(2) *= fy;
		width=lroundf(width*fx);
		height=lroundf(height*fy);		
        UpdateParameters();
    }


    // Epipole d'une autre camera dans cette camera
    Vector3 Epipole(const Camera &cam) const {
        return (*this) * cam.Center();
    }


    // Matrice fondamentale d'une paire de cameras
    Matrix3x3 Fondamental(const Camera &cam) const {
        return Matrix3x3::CrossProd(Epipole(cam)) * matrix * cam.matrix.Inverse();
    }


    // Normalisation de la matrice de camera : la troisieme ligne doit etre de norme 1
    void Normalize() {
        const float n = matrix.GetRow(3).Norm();
        matrix = matrix / n;
        vector = vector / n;
    }
	
	
	void Rigide2GL( double res[16]) {
		Matrix3x4  inv = rigide.Inverse();
		res[0+4*0]=inv.matrix(1,1); res[1+4*0]=inv.matrix(2,1); res[2+4*0]=inv.matrix(3,1); res[3+4*0]=0;
		res[0+4*1]=inv.matrix(1,2); res[1+4*1]=inv.matrix(2,2); res[2+4*1]=inv.matrix(3,2); res[3+4*1]=0;	
		res[0+4*2]=inv.matrix(1,3); res[1+4*2]=inv.matrix(2,3); res[2+4*2]=inv.matrix(3,3); res[3+4*2]=0;	
		res[0+4*3]=inv.vector(1); 
		res[1+4*3]=inv.vector(2); 
		res[2+4*3]=inv.vector(3); 
		res[3+4*3]=inv.matrix.GetRow(3).Norm();
	}	
	
	void display(){
		cout <<  "Original Projection :" << endl;
		for(int i=1;i<4;i++)
			cout << matrix(i,1) << " " << matrix(i,2) << " " << matrix(i,3) << " " << rigide.vector(i) << endl;
		
		cout << "is decomposed in" << endl;
		cout <<  "Internal Matrix :" << endl;
		cout << alphau << " 0 " << deltau << endl;
		cout << "0 " << alphav << " "  <<deltav << endl;
		cout << "0 0 1" << endl;		

		cout <<  "External External :" << endl;
		for(int i=1;i<4;i++)
			cout << rigide.matrix(i,1) << " " << rigide.matrix(i,2) << " " << rigide.matrix(i,3) << " " << rigide.vector(i) << endl;
		
	}
};

#endif
