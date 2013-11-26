/*
 *  MVFlowWindow.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/01/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MV_TRANSFORM_PROPS_H
#define MV_TRANSFORM_PROPS_H

#include "ui_MVTransformProps.Qt4.h"
#include "MVData.h"

#include <QDialog>
#include <QCheckBox>
class MVTransformProps: public QDialog, public Ui::MVTransformProps {
	Q_OBJECT
public:
	MVData * data;
	MVTransformProps(QWidget *parent, MVData *_data): QDialog(parent)
	{
		setupUi(this);
		data=_data;
		float *bounds = data->mesh.getBoundingBox();
		label_X1->setText(QString("[%1,").arg(bounds[0]));
		label_X2->setText(QString("%1]").arg(bounds[3]));
		label_Y1->setText(QString("[%1,").arg(bounds[1]));
		label_Y2->setText(QString("%1]").arg(bounds[4]));
		label_Z1->setText(QString("[%1,").arg(bounds[2]));
		label_Z2->setText(QString("%1]").arg(bounds[5]));

	}

	public slots:

	
	void on_radioMove_toggled(bool checked) {
		groupBoxMove->setEnabled(checked);
	}

	void on_radioRotate_toggled(bool checked) {
		groupBoxRotate->setEnabled(checked);
	}

	void on_radioScale_toggled(bool checked) {
		groupBoxScale->setEnabled(checked);
	}


	////////////////////////////////////////////////////////////////////////////////////
	void on_buttonOk_accepted() {
		//magic
		if (radioMove->isChecked()) {
			cout << "moving" << endl;
			data->mesh.move(doubleMoveX->value(),doubleMoveY->value(),doubleMoveZ->value());
		}
		else if (radioRotate->isChecked()) {

			Affine_3 rx,ry,rz,r;
			double ax= doubleRotateX->value()/180*PI;
			double ay= doubleRotateY->value()/180*PI;
			double az= doubleRotateZ->value()/180*PI;

			rx = Affine_3(1,0,0,  0,cos(ax),sin(ax), 0,-sin(ax),cos(ax));
			ry = Affine_3(cos(ay),0,-sin(ay),0,1,0, sin(ay),0,cos(ay));
			rz = Affine_3(cos(az), sin(az),0,-sin(az),cos(az),0,0,0,1);

			r = rx*ry*rz;

			cout << "rotate" << endl;
			data->mesh.rotate(r,radioRotateMeshOrigin->isChecked());
		}
		else if(radioScale->isChecked()) {
			cout << "scaling" << endl;
			data->mesh.scale(doubleScaleX->value(),doubleScaleY->value(),doubleScaleZ->value());
		}

	}
	
	
};

#endif
