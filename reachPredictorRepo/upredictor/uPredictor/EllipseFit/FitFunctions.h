#ifndef FIT_FUNCTIONS_H
#define FIT_FUNCTIONS_H

#include <Eigen/Core>

//Points are assumed to be 1 pt per column in Cartesian format 2D
Eigen::MatrixXd buildConicConstraintMat(Eigen::MatrixXd inPts);
<<<<<<< HEAD
Eigen::MatrixXd buildConicConstraintMatHom(Eigen::MatrixXd inPtsHom);
=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
Eigen::MatrixXd arrangeConicVectIntoImplicit(Eigen::MatrixXd inVect);
Eigen::MatrixXd aThirtyThreeFromConicSolution(Eigen::MatrixXd inVect);
Eigen::MatrixXd twoDHomToThreeDHom(Eigen::MatrixXd inMat, double zOff);
Eigen::MatrixXd cart3DRotMatToHomRT(Eigen::MatrixXd inR);

<<<<<<< HEAD

=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
#endif
