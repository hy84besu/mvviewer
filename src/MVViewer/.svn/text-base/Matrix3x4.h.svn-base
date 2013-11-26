#ifndef MATRIX3X4_H
#define MATRIX3X4_H

#include <iostream>
#include <fstream>
#include "Matrix3x3.h"
#include "Vector3.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Classe representant une matrice 3*4
class Matrix3x4 {
public:
	Matrix3x3 matrix; // Matrice 3x3
	Vector3 vector;   // Vecteur colonne


	// Destructeur
	virtual ~Matrix3x4() {}


	// Constructeur par defaut
	Matrix3x4() {}


	// Constructeur par recopie
	Matrix3x4 & operator = (const Matrix3x4 &a) {
		matrix.operator = (a.matrix);
		vector.operator = (a.vector);
		return *this;
	}


	// Matrice nulle
	void Zero() {
		matrix.Zero();
		vector.Zero();
	}


	// Matrice 'identite'
	void Identity() {
		matrix.Identity();
		vector.Zero();
	}


	// Composition
	Matrix3x4 Composition(const Matrix3x4 &a) {
		Matrix3x4 b;
		b.matrix = matrix * a.matrix;
		b.vector = vector + matrix * a.vector;
		return b;
	}


    // Inverse
    Matrix3x4 Inverse() const {
        Matrix3x4 r;
        r.matrix = matrix.Inverse();
        r.vector = - r.matrix * vector;
        return r;
    }


	// Oppose
	Matrix3x4 operator - () const {
		Matrix3x4 m;
        m.matrix = - matrix;
        m.vector = - vector;
        return m;
	}


	// Multiplication par un vecteur
	Vector3 operator * (const Vector3 &v) const {
		return matrix * v + vector;
	}

    
	// Multiplication par un scalaire
	Matrix3x4 operator * (float x) const {
		Matrix3x4 m;
        m.matrix = matrix * x;
        m.vector = vector * x;
        return m;
	}


	// Division par un scalaire
	Matrix3x4 operator / (float x) const {
		Matrix3x4 m;
        m.matrix = matrix / x;
        m.vector = vector / x;
        return m;
	}


	// Version rapide de la multiplication
	void FastMultiply(float x, float y, float z, float &u, float &v, float &w) const {
		matrix.FastMultiply(x,y,z,u,v,w);
		u += vector.coord[0];
		v += vector.coord[1];
		w += vector.coord[2];
	}


	// Entrees / Sorties
	int Load(const char *name = "-") {
		std::ifstream file(name);
		if (!file) {
			std::cerr << "Error: Unable to read '" << name << "'" << std::endl;
			return 1;
		}
		return Read(file);
	}

	int Save(const char *name = "-") const {
		std::ofstream file(name);
		if (!file) {
			std::cerr << "Error : Unable to write '" << name << "'" << std::endl;
			return 1;
		}
		return Write(file);
	}

	virtual int Read(std::istream &in) {
		for (int i=1;i<=3;i++) {
			for (int j=1;j<=3;j++)
				in >> matrix(i,j);
			in >> vector(i);
		}
		return 0;
	}

	virtual int Write(std::ostream &out) const {
		for (int i=1;i<=3;i++) {
			for (int j=1;j<=3;j++)
				out << matrix(i,j) << " ";
			out << vector(i) << std::endl;
		}
		return 0;
	}
};

#endif
