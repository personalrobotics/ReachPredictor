#include "LinearAlgebra.h"

#include <iostream>

using namespace Eigen;
using namespace std;

<<<<<<< HEAD

Eigen::MatrixXd centerPts 
	(Eigen::MatrixXd inPts3dRowWise){
	size_t n = inPts3dRowWise.rows();

	MatrixXd retPts3dCentered = MatrixXd::Zero(n,3);
	retPts3dCentered = inPts3dRowWise.rowwise() 
		- inPts3dRowWise.colwise().mean();
	return retPts3dCentered;
}

//Eigen: V is a matrix with the 
//eigenvectors as its columns
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
	solveEigensystem(Eigen::MatrixXd a) {
	
	
=======
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
	solveEigensystem(Eigen::MatrixXd a) {
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
	EigenSolver<MatrixXd> eigensolver(a);

	//cout << eigensolver.eigenvalues() << endl;
	//cout << eigensolver.eigenvectors() << endl;

	//SelfAdjointEigenSolver<MatrixXd> eigensolver(a);

	std::map<double, MatrixXd> retVal;

	for(size_t i = 0; i < a.rows(); i++)
		retVal.insert(
			pair<double, MatrixXd>(
				eigensolver.eigenvalues()(i).real()
				, eigensolver.eigenvectors().real().block(0, i, a.rows(), 1)));

	MatrixXd eigenValues(a.rows(), 1);
	MatrixXd eigenVectors(a.rows(), a.rows());
	size_t i = 0;
<<<<<<< HEAD
	for(std::map<double,
		Eigen::MatrixXd>::reverse_iterator iter =
			retVal.rbegin(); iter != retVal.rend(); iter++) {
		
		eigenValues(i,0) = iter->first;
		//col-stacking Eig. vectors 
=======
	for(std::map<double, Eigen::MatrixXd>::reverse_iterator iter =
			retVal.rbegin();
		iter != retVal.rend(); iter++) {
		eigenValues(i,0) = iter->first;
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
		eigenVectors.block(0,i,a.rows(),1) = iter->second;
		i++;
	}

	return std::pair<MatrixXd, MatrixXd>(eigenValues, eigenVectors);
}

<<<<<<< HEAD
//X is a n by 3 matrix in which  all the 3D coordinates of the points are row-stacked.
// X_mean is the mean coordinate, i.e., the centroid of the points. 
MatrixXd buildCovMatPCA (MatrixXd inPts3dRowWise){
	
	size_t n = inPts3dRowWise.rows();
	MatrixXd inPts3dCentered = 
		centerPts (inPts3dRowWise) ;

	MatrixXd covMat = MatrixXd::Zero(3,3);	
	covMat = inPts3dCentered.transpose()
		* inPts3dCentered
		/ (n-1.0);

	return covMat;
}

// diagEigMat: the diagonal matrix  with the 3  eigenvalues of covMat
Eigen::MatrixXd getDiagEigMat
	(Eigen::MatrixXd inPts3dRowWise) {

	MatrixXd covMat = MatrixXd::Zero(3,3);
	covMat = buildCovMatPCA(inPts3dRowWise);

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigenSys 
		= solveEigensystem(covMat); 

	MatrixXd retDiagEigMat = MatrixXd::Identity(3,3);
	for (int i =0; i<3; i++)
		retDiagEigMat(i,i) = eigenSys.first(i,0);
	
	return retDiagEigMat;
}

// OrthEigVMat:orthogonal matrix row-stacking the corresponding eigenvectors
Eigen::MatrixXd getOrthEigVectsMatRowStacked
	(Eigen::MatrixXd inPts3dRowWise) {

	MatrixXd covMat = MatrixXd::Zero(3,3);
	covMat = buildCovMatPCA(inPts3dRowWise);

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigenSys 
	= solveEigensystem(covMat); 
	return eigenSys.second.transpose();
}

// nx3
Eigen::MatrixXd allignPtsToPCA
	(Eigen::MatrixXd inPts3dRowWise) {

	Eigen::MatrixXd EigVectsMatRowStacked = 
		getOrthEigVectsMatRowStacked(inPts3dRowWise);
	// nx3
	MatrixXd ptsPCAaligned = 
		MatrixXd::Zero(inPts3dRowWise.rows(),3);
	ptsPCAaligned = centerPts(inPts3dRowWise) 
		* EigVectsMatRowStacked.transpose();

	return ptsPCAaligned;
}

/** Eqn 23 - Quinn **/
//inPts3dRowWise (nx3) 
Eigen::MatrixXd buildPCAGausNoiseVarEigVects		
	(Eigen::MatrixXd inPts3dRowWise){

  MatrixXd covMat = buildCovMatPCA(inPts3dRowWise);
	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigenSys 
		= solveEigensystem(covMat); 		

	MatrixXd GaussE = MatrixXd::Zero(3,1);

	for (int i = 0; i<3; i++)	{
		GaussE (i,0) = 4 * eigenSys.first(i,0) 
			* eigenSys.first(2,0) / (inPts3dRowWise.rows()-2);
	}	
	return GaussE;
}

//alglib::ae_int_t df1, alglib::ae_int_t df2,
//Eigen::MatrixXd buildErrorBoundsPCAaligned(Eigen::MatrixXd inPts3dRowWise, double alpha)	{
//inPts3dRowWise (nx3) 
double getInvFdist (Eigen::MatrixXd inPts3dRowWise, double alpha){

	alglib::ae_int_t df1 = (alglib::ae_int_t) 2;
	alglib::ae_int_t df2 = 
		(alglib::ae_int_t) inPts3dRowWise.rows()-df1;

	double FisherTestDensity = 
		alglib::invfdistribution(df1, df2, alpha);
	
	return FisherTestDensity;

}

//inPts3dRowWise (nx3)
Eigen::MatrixXd buildEigVectsErrs
	(Eigen::MatrixXd inPts3dRowWise){

	MatrixXd noiseVar = MatrixXd::Zero(3,1);
	noiseVar = buildPCAGausNoiseVarEigVects(inPts3dRowWise);

	double FisherTestDensit = 
		getInvFdist(inPts3dRowWise, 0.025);

	Eigen::MatrixXd retEigVectsErrs = 
		MatrixXd::Zero(3,1);
	retEigVectsErrs = noiseVar * FisherTestDensit;

	return retEigVectsErrs;
}

Eigen::MatrixXd getHypErrShellAxes
	(Eigen::MatrixXd inPts3dRowWise){
	
	//h
	Eigen::MatrixXd hyperboloidAxes = 
		MatrixXd:: Zero(3,1);
	//e_λ
	Eigen::MatrixXd eigVectsErrs = 
		MatrixXd::Zero(3,1);
	eigVectsErrs = buildEigVectsErrs(inPts3dRowWise);
	//λ
	Eigen::MatrixXd diagEigValsMat = 
		MatrixXd::Zero(3,3);
	diagEigValsMat = getDiagEigMat(inPts3dRowWise);


	hyperboloidAxes(0,0) = 
		diagEigValsMat(0,0) - eigVectsErrs(0,0);
	hyperboloidAxes(1,0) =
		diagEigValsMat(1,1) - eigVectsErrs (1,0);
	hyperboloidAxes(2,0) = 
		diagEigValsMat(2,2) + eigVectsErrs (2,0);
	
	return hyperboloidAxes;
}

Eigen::MatrixXd getHypErrBounds
	(Eigen::MatrixXd inPts3dRowWise) {
	
	int nRows = inPts3dRowWise.rows();
	MatrixXd retHypErrBounds = 
		MatrixXd::Zero(nRows,2);

	//Hyperblois Axes    
	Eigen::MatrixXd ErrHyperboloidAxes =
		MatrixXd::Zero(3,1);
	ErrHyperboloidAxes = 
		getHypErrShellAxes(inPts3dRowWise);

	//inPts3DPCAaligned (nx3)
	Eigen::MatrixXd inPts3DPCAaligned = 
		allignPtsToPCA(inPts3dRowWise);

	for (int i=0; i<nRows; i++){
		retHypErrBounds(i,0) = ErrHyperboloidAxes(2,0) 
			* sqrt(pow((inPts3DPCAaligned(i,0)
					/ErrHyperboloidAxes(0,0)),2.0) + 1);

		retHypErrBounds(i,1) = ErrHyperboloidAxes(2,0) 
			* sqrt(pow((inPts3DPCAaligned(i,1)
					/ErrHyperboloidAxes(1,0)),2.0) + 1);
	}
	return retHypErrBounds;
}


/** Eqn 35 - Quinn **/
std::pair<double, double> getMaxMinAngularErrs
	(Eigen::MatrixXd hErrAxes){
	
	double minTheta = 2.0 
		* atan(hErrAxes(2,0)/hErrAxes(1,0));
	double maxTheta = 2.0 
		* atan(hErrAxes(2,0)/hErrAxes(0,0));

	return std::pair<double, double>
		(maxTheta, minTheta);

}

=======
//X is a 3 by n matrix in which  all the 3D coordinates of the 
//points are row-stacked.
// X_mean is the mean coordinate, i.e., the centroid of the points. 
MatrixXd buildCovMatPCA(MatrixXd inPts3D){
	size_t n = inPts3D.cols();
	MatrixXd covMat = MatrixXd::Zero(3,3);	
	covMat = (inPts3D.colwise() - inPts3D.rowwise().mean())
	* (inPts3D.colwise() - inPts3D.rowwise().mean()).transpose()
	/ n;

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigenSys 
	= solveEigensystem(covMat); 

	// diagEigMat: the diagonal matrix  with the 3  eigenvalues of covMat
	// OrthEigVMat:orthogonal matrix row-stacking the corresponding eigenvectors
	MatrixXd diagEigMat = MatrixXd::Identity(3,3);
	MatrixXd OrthEigVMat = MatrixXd::Identity(3,3);
	for (int i =0; i<3; i++)
		diagEigMat(i,i) = eigenSys.first(i,0);

	OrthEigVMat = eigenSys.second.transpose();

	MatrixXd diagCov = OrthEigVMat * diagEigMat * OrthEigVMat.transpose();

	//std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigSysDiagCov
	//= solveEigensystem(diagCov); 
	//cout << "diagEigMat:\n" << diagEigMat << endl;
	//cout << "OrthEigVMat:\n" << OrthEigVMat << endl;
	return diagCov;
}


>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
