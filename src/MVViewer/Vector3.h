#ifndef VECTOR3_H
#define VECTOR3_H

//#include <cmath>
#include <math.h>
#include <iostream>
#include <cassert>


//////////////////////////////////////////////////////////////////////////////////////////////////
// Classe representant un vecteur de taille 3
class Vector3 {
public:
	// Composantes du vecteur
	double coord[3];


	// Constructeur par defaut
	Vector3() {
		Zero();
	}


	// Constructeur avec trois composantes
	Vector3(double u, double v, double w) {
		coord[0] = u;
		coord[1] = v;
		coord[2] = w;
	}


	// Constructeur par recopie
	Vector3 & operator = (const Vector3 &v) {
		for (int i=0;i<3;i++) coord[i] = v.coord[i];
		return *this;
	}


	// Acces aux composantes
	double& operator () (int i) {
		assert(i>=1 && i<=3);
		return coord[i-1];
	}

	const double& operator () (int i) const {
		assert(i>=1 && i<=3);
		return coord[i-1];
	}


	// Vecteur nul
	void Zero() {
		for (int i=0;i<3;i++) coord[i] = 0;
	}


	// Oppose
	Vector3 operator - () const {
		Vector3 w;
		for (int i=0;i<3;i++) w.coord[i] = - coord[i];
		return w;
	}
	
	// equality 
	bool operator == (const Vector3 &w) const {
		bool isEqual = true;
		for (int i=0;i<3;i++) isEqual = isEqual && (w.coord[i] == coord[i]);
		return isEqual;
	}	


	// Addition
	Vector3 operator + (const Vector3 &v) const {
		Vector3 w;
		for (int i=0;i<3;i++) w.coord[i] = coord[i] + v.coord[i];
		return w;
	}


	// Soustraction
	Vector3 operator - (const Vector3 &v) const {
		Vector3 w;
		for (int i=0;i<3;i++) w.coord[i] = coord[i] - v.coord[i];
		return w;
	}


	// Multiplication par un scalaire
	Vector3 operator * (double x) const {
		Vector3 w;
		for (int i=0;i<3;i++) w.coord[i] = coord[i] * x;
		return w;
	}


	// Division par un scalaire
	Vector3 operator / (double x) const {
		Vector3 w;
		for (int i=0;i<3;i++) w.coord[i] = coord[i] / x;
		return w;
	}


	// Produit scalaire
	double operator * (const Vector3 &v) const {
		double sum = 0;
		for (int i=0;i<3;i++) sum += coord[i] * v.coord[i];
		return sum;
	}


	// Produit vectoriel - cross product
	Vector3 operator ^ (const Vector3 &v) const {
		Vector3 w;
		w.coord[0] = coord[1] * v.coord[2] - coord[2] * v.coord[1];
		w.coord[1] = coord[2] * v.coord[0] - coord[0] * v.coord[2];
		w.coord[2] = coord[0] * v.coord[1] - coord[1] * v.coord[0];
		return w;
	}


	// Norme
	double Norm() const {
		return sqrt( operator * (*this) );
	}

	// L'angle en Radians
	double AngleWith(Vector3 b) {
		if ((b.Norm() == 0) || (Norm() == 0)) {
			printf("invalid norm in AngleWith\n");
			return -1;
		}
		Vector3 a1 = *this / Norm();
		Vector3 a2 = b / b.Norm();
		return acos(a1*a2);
	}
	// L'angle en Radians
	double AngleWithDeg(Vector3 b) {
		return AngleWith(b)*180 / 3.14159265;
	}

    
    // Entrees sorties
    int read(std::istream &in) {
		for (int i=0;i<3;i++) in >> coord[i];
		return 0;
	}
	
    int write(std::ostream &out) const {
		out << "(";
		for (int i=0;i<3;i++) {
			out << coord[i];
			if (i!=3) out << ",";
		}
		out << " )";
		return 0;
	}
};

#endif
