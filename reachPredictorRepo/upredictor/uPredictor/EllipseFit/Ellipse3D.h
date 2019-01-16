#ifndef ELLIPSE3D_H
#define ELLIPSE3D_H

#include "Ellipse.h"
#include "EllipseImplicitFit.h"
#include "EllipseThetaMin.h"

#include <Eigen/Dense>

//template<typename T>
class Ellipse3D {
public:
	Ellipse3D(MatrixXd inPtsAlongColsHom);

	~Ellipse3D();

	EllipseImplicitFit getEIF();

	Ellipse getEllipse();

	double getTotalXYCost();
	MatrixXd getXYCostMat();

	MatrixXd getPointAtThetaH(double theta);

	MatrixXd getPointAtThetasH(MatrixXd thetas);

<<<<<<< HEAD
	pair<double, double> findOneTheta
		(double startTheta, double x, double y);
=======
	pair<double, double> findOneTheta(double startTheta, double x, double y);
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7

	pair<double, double> findOneThetaQuad(
		double &startTheta, double x, double y);

	MatrixXd findThetas();

<<<<<<< HEAD
	MatrixXd ellipticalInterpolator(size_t sliceN);
	

=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
protected:
	EllipseImplicitFit _eif;
	Ellipse _ellipse;
	double _totalXYCost;
	MatrixXd _XYCosts;
};

#endif
