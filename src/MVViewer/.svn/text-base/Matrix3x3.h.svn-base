#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include <iostream>
#include <cassert>
#include "Vector3.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Classe representant une matrice 3*3
class Matrix3x3 {
public:
	// Coefficients de la matrice
	float coeff[3][3];

	// Constructeur par defaut
	Matrix3x3() {
		Zero();
	}


	// Constructeur par recopie
	Matrix3x3 & operator = (const Matrix3x3 &a) {
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) coeff[i][j] = a.coeff[i][j];
		return *this;
	}


	// Acces aux coefficients
	float& operator () (int i, int j) {
		assert(i>=1 && i<=3 && j>=1 && j<=3);
		return coeff[i-1][j-1];
	}

	const float& operator () (int i, int j) const {
		assert(i>=1 && i<=3 && j>=1 && j<=3);
		return coeff[i-1][j-1];
	}

	// Matrice nulle
	void Zero() {
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) coeff[i][j] = 0;
	}


	// Matrice identite
	void Identity() {
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) coeff[i][j] = (i==j) ? 1.0f:0;
	}


	// Oppose
	Matrix3x3 operator - () const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = - coeff[i][j];
		return b;
	}


	// Addition
	Matrix3x3 operator + (const Matrix3x3 &a) const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = coeff[i][j] + a.coeff[i][j];
		return b;
	}


	// Soustraction
	Matrix3x3 operator - (const Matrix3x3 &a) const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = coeff[i][j] - a.coeff[i][j];
		return b;
	}


	// Multiplication par un scalaire
	Matrix3x3 operator * (float x) const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = coeff[i][j] * x;
		return b;
	}


	// Division par un scalaire
	Matrix3x3 operator / (float x) const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = coeff[i][j] / x;
		return b;
	}


	// Produit de deux matrices
	Matrix3x3 operator * (const Matrix3x3 &a) const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k=0;k<3;k++) b.coeff[i][j] += coeff[i][k] * a.coeff[k][j];
		return b;
	}


	// Produit matrice vecteur
	Vector3 operator * (const Vector3 &v) const {
		Vector3 w;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) w.coord[i] += coeff[i][j] * v.coord[j];
		return w;
	}


	// Transposee d'une matrice
	Matrix3x3 Transpose() const {
		Matrix3x3 b;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) b.coeff[i][j] = coeff[j][i];
		return b;
	}

    
    // Inverse d'une matrice
    Matrix3x3 Inverse() const {
        const float det = Det();
        Matrix3x3 b;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
            const int ia = (i+1)%3;
            const int ib = (i+2)%3;
            const int ja = (j+1)%3;
            const int jb = (j+2)%3;
            b.coeff[j][i] = ( coeff[ia][ja] * coeff[ib][jb] - coeff[ib][ja] * coeff[ia][jb] ) / det;
        }
        return b;
    }


    // Determinant
    float Det() const {
        return
            coeff[0][0] * (coeff[1][1] * coeff[2][2] - coeff[1][2] * coeff[2][1])
            - coeff[1][0] * (coeff[0][1] * coeff[2][2] - coeff[2][1] * coeff[0][2])
            + coeff[2][0] * (coeff[0][1] * coeff[1][2] - coeff[1][1] * coeff[0][2]);
    }


	// Manipulation des lignes et des colonnes
	Vector3 GetColumn(int j) const {
		assert(j>=1 && j<=3);
		Vector3 v;
		for (int i=0;i<3;i++) v.coord[i] = coeff[i][j-1];
		return v;
	}

	void SetColumn(int j, const Vector3 &v) {
		assert(j>=1 && j<=3);
		for (int i=0;i<3;i++) coeff[i][j-1] = v.coord[i];
	}

	Vector3 GetRow(int i) const {
		assert(i>=1 && i<=3);
		Vector3 v;
		for (int j=0;j<3;j++) v.coord[j] = coeff[i-1][j];
		return v;
	}

	void SetRow(int i, const Vector3 &v) {
		assert(i>=1 && i<=3);
		for (int j=0;j<3;j++) coeff[i-1][j] = v.coord[j];
	}

	// Version rapide de la multiplication
	void FastMultiply(float x, float y, float z, float &u, float &v, float &w) const {
		u = coeff[0][0] * x + coeff[0][1] * y + coeff[0][2] * z;
		v = coeff[1][0] * x + coeff[1][1] * y + coeff[1][2] * z;
		w = coeff[2][0] * x + coeff[2][1] * y + coeff[2][2] * z;
	}

	
    // Entrees sorties
    int Read(std::istream &in) {
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) in >> coeff[i][j];
		return 0;
	}
	
    int Write(std::ostream &out) const {
		for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) out << coeff[i][j] << " ";
			out << std::endl;
		}
		return 0;
	}


    // Matrice de produit vectoriel
    static Matrix3x3 CrossProd(const Vector3 &v) {
        Matrix3x3 a;
        a.Zero();
        a.coeff[1][0] = v.coord[2];
        a.coeff[0][1] = - v.coord[2];
        a.coeff[2][0] = - v.coord[1];
        a.coeff[0][2] = v.coord[1];
        a.coeff[2][1] = v.coord[0];
        a.coeff[1][2] = - v.coord[0];
        return a;
    }
};

#endif
