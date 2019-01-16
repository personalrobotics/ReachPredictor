#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

<<<<<<< HEAD
#include <stdio.h>
#include <sys/time.h>
#include <math.h> 
#include "linalg.h"
#include "specialfunctions.h"

=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
#include <Eigen/Dense>

#include <map>

<<<<<<< HEAD
Eigen::MatrixXd centerPts 
	(Eigen::MatrixXd inPts3dRowWise);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> 
	solveEigensystem(Eigen::MatrixXd a);

Eigen::MatrixXd buildCovMatPCA
	(Eigen::MatrixXd inPts3dRowWise);

// diagEigMat: the diagonal matrix  with the 3  eigenvalues of covMat
Eigen::MatrixXd getDiagEigMat
	(Eigen::MatrixXd inPts3dRowWise);

// OrthEigVMat:orthogonal matrix row-stacking the corresponding eigenvectors
Eigen::MatrixXd getOrthEigVectsMatRowStacked
	(Eigen::MatrixXd inPts3dRowWise);

Eigen::MatrixXd allignPtsToPCA
	(Eigen::MatrixXd inPts3dRowWise);

/** Eqn 23 - Quinn **/
Eigen::MatrixXd buildPCAGausNoiseVarEigVects
	(Eigen::MatrixXd inPts3dRowWise);

double getInvFdist 
	(Eigen::MatrixXd inPts3dRowWise, double alpha);

/** Eqn 18 - Quinn **/
Eigen::MatrixXd buildEigVectsErrs
	(Eigen::MatrixXd inPts3dRowWise);

//alglib::ae_int_t df1, alglib::ae_int_t df2,
Eigen::MatrixXd getHypErrShellAxes
	(Eigen::MatrixXd inPts3dRowWise);

Eigen::MatrixXd getHypErrBounds
	(Eigen::MatrixXd inPts3dRowWise);

/** Eqn 35 - Quinn **/
std::pair<double, double> getMaxMinAngularErrs
	(Eigen::MatrixXd hErrAxes);


=======
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> solveEigensystem(Eigen::MatrixXd a);
Eigen::MatrixXd buildCovMatPCA(Eigen::MatrixXd inPts3D);
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
#endif
