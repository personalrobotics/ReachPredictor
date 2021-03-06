#include "FitFunctions.h"

using namespace Eigen;

MatrixXd buildConicConstraintMat(MatrixXd inPts) {
	size_t nPts = inPts.cols();
	MatrixXd constraintMat(nPts, 6);
	for(size_t i = 0; i < nPts; i++) {
		double x = inPts(0, i);
		double y = inPts(1, i);
		constraintMat(i, 0) = x * x;
		constraintMat(i, 1) = x * y;
		constraintMat(i, 2) = y * y;
		constraintMat(i, 3) = x;
		constraintMat(i, 4) = y;
		constraintMat(i, 5) = 1.0;
	}

	return constraintMat;
}

<<<<<<< HEAD

MatrixXd buildConicConstraintMatHom(MatrixXd inPtsHom) {
	size_t nPts = inPtsHom.cols();
	MatrixXd constraintMat(nPts, 6);
	for(size_t i = 0; i < nPts; i++) {
		double x_1 = inPtsHom(0, i);
		double x_2 = inPtsHom(1, i);
		double x_3 = inPtsHom(2, i);
		constraintMat(i, 0) = x_1 * x_1;
		constraintMat(i, 1) = x_1 * x_2;
		constraintMat(i, 2) = x_2 * x_2;
		constraintMat(i, 3) = x_1 * x_3;
		constraintMat(i, 4) = x_2 * x_3;
		constraintMat(i, 5) = x_3 * x_3;;
	}

	return constraintMat;
}

=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
MatrixXd arrangeConicVectIntoImplicit(MatrixXd inVect) {
	MatrixXd retVal(3,3);
	double a = inVect(0,0);
	double b = inVect(1,0);
	double c = inVect(2,0);
	double d = inVect(3,0);
	double e = inVect(4,0);
	double f = inVect(5,0);
	retVal <<	a,		b/2.0,	d/2.0
			,	b/2.0,	c,		e/2.0
			,	d/2.0,	e/2.0,	f;
	return retVal;
}

Eigen::MatrixXd aThirtyThreeFromConicSolution(Eigen::MatrixXd inVect) {
	Eigen::MatrixXd retVal(2,2);
	retVal <<	inVect(0,0), inVect(1,0) / 2.0
			,	inVect(1,0) / 2.0, inVect(2,0);
	return retVal;
}

MatrixXd twoDHomToThreeDHom(MatrixXd inMat, double zOff) {
	MatrixXd retVal(4, inMat.cols());
	retVal.block(0,0,2,inMat.cols()) = inMat.block(0,0,2,inMat.cols());
	retVal.block(2,0,1,inMat.cols()) =
		MatrixXd::Constant(1, inMat.cols(), zOff);
	retVal.block(3,0,1,inMat.cols()) = inMat.block(2,0,1,inMat.cols());
	return retVal;
}

MatrixXd cart3DRotMatToHomRT(Eigen::MatrixXd inR) {
	MatrixXd rt = Matrix<double,4,4>::Zero();
	rt.block(0,0,3,3) = inR;
	rt(3,3) = 1.0;
	return rt;
}

<<<<<<< HEAD







=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
