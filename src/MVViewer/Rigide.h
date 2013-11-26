#ifndef RIGIDE_H
#define RIGIDE_H

#include "Matrix3x4.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Classe pour representer une transformation rigide
class Rigide : public Matrix3x4 {
public:
	float alpha, beta, gamma; // Les trois angles de rotation


	// Destructeur
	virtual ~Rigide() {}

	// Constructeur par defaut
	Rigide() {
		Identity();
	}


	// Constructeur a partir des trois angles de rotation et de la translation
	Rigide(float _alpha, float _beta, float _gamma, float _tx, float _ty, float _tz) : alpha(_alpha), beta(_beta), gamma(_gamma) {
		UpdateMatrix();
		vector = Vector3(_tx,_ty,_tz);
	}


	// Constructeur par recopie
	Rigide & operator = (const Rigide &a) {
		Matrix3x4::operator = (a);
		alpha = a.alpha;
		beta = a.beta;
		gamma = a.gamma;
		return *this;
	}

	Rigide & operator = (const Matrix3x4 &a) {
		Matrix3x4::operator = (a);
		UpdateAngle();
		return *this;
	}

	// Identite
	void Identity() {
		alpha = beta = gamma = 0;
		Matrix3x4::Identity();
	}


	// Entrees sorties
	// Apres avoir lu la matrice il faut en deduire les trois angles
	virtual int Read(std::istream &in) {
		int ret = Matrix3x4::Read(in);
		if (ret) return ret;
		UpdateAngle();
		return 0;
	}


	// Mise a jour de la matrice a partir des angles
	void UpdateMatrix() {
		const float coa = cos(alpha);
		const float sia = sin(alpha);
		const float cob = cos(beta);
		const float sib = sin(beta);
		const float cog = cos(gamma);
		const float sig = sin(gamma);

		matrix(1,1) = cog * cob;
		matrix(1,2) = cog * sib * sia - sig * coa;
		matrix(1,3) = cog * sib * coa + sig * sia;
		matrix(2,1) = sig * cob;
		matrix(2,2) = sig * sib * sia + cog * coa;
		matrix(2,3) = sig * sib * coa - cog * sia;
		matrix(3,1) = -sib;
		matrix(3,2) = cob * sia;
		matrix(3,3) = cob * coa;
	}


	// Mise a jour des angles a partir de la matrice
	void UpdateAngle() {
		std::cerr << "TODO: Rigide::UpdateAngle()" << std::endl;
	}
};

#endif
