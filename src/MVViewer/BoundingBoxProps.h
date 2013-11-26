/*
 *  BoundingBoxProps.h
 *  MVStereo
 *
 *  Created by Andrei Zaharescu on 13/03/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef BOUNDING_BOX_PROPS_H
#define BOUNDING_BOX_PROPS_H

#include "ui_BoundingBoxProps.h"
#include "MVData.h"

#include <QDialog>
#include <QCheckBox>
class BoundingBoxProps: public QDialog, public Ui::BoundingBoxProps {
	Q_OBJECT
public:
	MVData * data;
	BoundingBoxProps(QWidget *parent, MVData *_data): QDialog(parent)
	{
		setupUi(this);
		data=_data;
		float *bounds = data->mesh.getBoundingBox();
		doubleBoxX1->setValue(bounds[0]);
		doubleBoxY1->setValue(bounds[1]);
		doubleBoxZ1->setValue(bounds[2]);		
		doubleBoxX2->setValue(bounds[3]);
		doubleBoxY2->setValue(bounds[4]);
		doubleBoxZ2->setValue(bounds[5]);		
	}
	
	public slots:
	
	
	
	////////////////////////////////////////////////////////////////////////////////////
	void on_buttonOk_accepted() {
		cout << "Restricting to bbox values" << endl;
		data->mesh.restrictToBoundingBox(doubleBoxX1->value(),doubleBoxY1->value(),doubleBoxZ1->value(),doubleBoxX2->value(),doubleBoxY2->value(),doubleBoxZ2->value());
		
	}
	
	
};

#endif
