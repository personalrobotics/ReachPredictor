
#include "ParseMathematica.h"
#include "ParseCSV.h"
#include "UBCUtil.h"
#include "FitFunctions.h"
#include "Ellipse3D.h"
<<<<<<< HEAD
#include "Plane.h"
=======

>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7

#include "LinearAlgebra.cpp"
#include "RigidTrans2D.h"

#include "BaysFitFunctions.h"
<<<<<<< HEAD
#include <math.h> 
#include <Eigen/Core>
=======

>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
#include <Eigen/Dense>
//#include <Eigen/SVD>

//#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

MatrixXd fixThetas(MatrixXd a) {
	MatrixXd retVal(a.rows(), a.cols());

	double curTheta = a(0,0);
	while(curTheta >= -M_PI) curTheta -= M_PI;
	while(curTheta <= M_PI) curTheta += M_PI;
	retVal(0,0) = curTheta;

	for(size_t i = 1; i < a.cols(); i++) {
		double curTheta = a(0,i);
		while(curTheta >= -M_PI) curTheta -= 2.0 * M_PI;
		while(curTheta <= M_PI) curTheta += 2.0 * M_PI;

		double testA = curTheta + 2.0 * M_PI;
		double testB = curTheta - 2.0 * M_PI;
		
		double distNull = curTheta - retVal(0, i - 1);
		double distA = testA - retVal(0, i - 1);
		double distB = testB - retVal(0, i - 1);

		retVal(0, i) = distNull < distA ? curTheta : testA;
		retVal(0, i) = distNull < distB ? curTheta : testB;
	}

	return retVal;
}

int main(int argc, char **argv) {
	srand(time(NULL));

<<<<<<< HEAD

	string fileName = argv[1];
	string reachCount = argv[2];

	string partId = getPartID(fileName);
	char reachCat = fileName.at((unsigned)(fileName.length()-5));

	int colNo = 0;
	//int nRows = countRowsFColCSV(argv[1]);

	std::vector<std::string> wristMarkers =
 		{"x", "y", "z"};
	int nRows = countRowsCSV(argv[1], wristMarkers[colNo]);

	// nx3
	MatrixXd inWPtsAlongRows(nRows,3);
	fillMatCSV(colNo, argv[1], wristMarkers[0], 
			wristMarkers[1], wristMarkers[2], inWPtsAlongRows);
	

	
	/*********** Ellipse Fitting ***********/
	
	Ellipse3D e3D(cartToHom(inWPtsAlongRows.transpose()));

	MatrixXd inPts2dRotatedAlongCols  =
		  e3D.getEIF().getPtsXYAlongCols();

	/*********** Plane Fitting ***********/


	/***Eigen System Solver***/
	Eigen::MatrixXd EigVectsMatRowStacked =
		MatrixXd::Zero(3,3);
	EigVectsMatRowStacked =
		getOrthEigVectsMatRowStacked (inWPtsAlongRows);	
	
	Eigen::MatrixXd diagEigValsMat = MatrixXd::Zero(3,3);
	diagEigValsMat = getDiagEigMat(inWPtsAlongRows);

	/***@Q The nominal Plane in PCA ***/
	Vector3d planeNvec = 
		EigVectsMatRowStacked.block(2,0,1,3).transpose();

	double planeStrike = atan2(planeNvec(0,0),planeNvec(1,0))
		- M_PI/2.0;
	double planeDip = acos(planeNvec(2,0)/planeNvec.norm());

	/****** My method - 
		Plane Normal Angles to XY *****/

	Vector3d PlaneNormalVectN = 
		e3D.getEIF().getPlane().getPlaneNormalVectN();
	Vector3d 	zAxis (0, 0, 1) ;
	double dotProduct = PlaneNormalVectN.dot(zAxis);
	Vector3d crossProductt = 
		PlaneNormalVectN.cross(zAxis);

	double planeAngleToXY = 
		atan2(crossProductt.norm(), dotProduct);

	/***Align to PCA***/
	// nx3
	MatrixXd inWPtsPCAaligned = MatrixXd::Zero(nRows,3);
	inWPtsPCAaligned	= allignPtsToPCA (inWPtsAlongRows);

	/***Error space - Hyperbloid Axes***/    
	Eigen::MatrixXd HypErrShellAxes = 
		MatrixXd::Zero(3,1);
	HypErrShellAxes = 
		getHypErrShellAxes(inWPtsAlongRows);


	/***Fixed-Length Normal Vector A.2
		Error space - ellipsoid major axes with a center
    offset √2h3 from the origin along the 3 axis. ***/    
	double fixedLEcenterOffset = 
		sqrt(2.0) *	HypErrShellAxes(2,0);
	Eigen::MatrixXd fixedLEllipsoidAxes  = 
		MatrixXd::Zero(3,1);
	
	fixedLEllipsoidAxes(0,0) = HypErrShellAxes(2,0) 
		* HypErrShellAxes(2,0) /HypErrShellAxes(0,0);
	fixedLEllipsoidAxes(1,0) = HypErrShellAxes(2,0) 
		* HypErrShellAxes(2,0) /HypErrShellAxes(0,0);
	fixedLEllipsoidAxes(2,0) = HypErrShellAxes(2,0);

	/***the hyperbolic error bounds***/
	MatrixXd HypErrBounds = MatrixXd::Zero(nRows,2);
	HypErrBounds = getHypErrBounds(inWPtsAlongRows);


	/***Max & Min Angular errors***/
	/* provides the angular width of the error 		
	distribution aligned with the major axes
	of the dataset.*/

	std::pair<double, double> maxMinAngularErrs = 
		getMaxMinAngularErrs(HypErrShellAxes);

	
	/** RAKE **/
	Vector3d v2 = 
		EigVectsMatRowStacked.block(1,0,1,3).transpose();
	Vector3d crossProduct = 
		planeNvec.cross(zAxis);

	double rake = acos(crossProduct.dot(v2));
/*********** End - Plane Fitting ***********/

	

=======
	//MatrixXd inPtsAlongColsHom = parseMathematica(getWholeFile(argv[1]));
	//cout << "inPtsAlongColsHom :\n" << inPtsAlongColsHom.rows()<< endl;

	int colNo = 0;
	int nRows = countRowsFColCSV(argv[1]);
	// 3xn
	MatrixXd inWPtsAlongCols(3,nRows);
	fillMatWholeCSV(colNo, argv[1], inWPtsAlongCols);
	//cout << "inWPtsAlongCols :\n" << inWPtsAlongCols<< endl;

	/*********** Plane Fitting ***********/
	//cout << "/*********** Plane Fitting ***********/\n"
	//vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>* eigenSysS;
  MatrixXd covMat = buildCovMatPCA(inWPtsAlongCols);
	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> eigenSys 
		= solveEigensystem(covMat); 		
	//cout << eigenSys.first << "\n" << endl;
	//cout << "EigenVals for " << argv[1] <<":\n" << eigenSys.first<< endl;
	/*********** End - Plane Fitting ***********/

	/*********** Ellipse Fitting ***********/
	Ellipse3D e3D(cartToHom(inWPtsAlongCols));
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
	MatrixXd thetas = e3D.findThetas();
	double totalXYCost = e3D.getTotalXYCost();
	MatrixXd fitCosts = e3D.getXYCostMat();
	MatrixXd pts2d = e3D.getEllipse().getPointAtThetasH(thetas);
	MatrixXd pts3d = e3D.getPointAtThetasH(thetas);
	
<<<<<<< HEAD
	size_t sliceN = 150;
	MatrixXd pts3dSim = 
		e3D.ellipticalInterpolator(sliceN);
	// 3xn
	//MatrixXd inPts2dRotatedAlongCols  = 
	//		e3D.getEIF().getPtsXYAlongCols();
=======
	// 3xn
	MatrixXd inPts2dRotatedAlongCols  = e3D.getEIF().getPtsXYAlongCols();
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7

	Ellipse ellipse = e3D.getEllipse();
	double cX = ellipse.getRT2D().getCX();
	double cY = ellipse.getRT2D().getCY();
	double alpha = ellipse.getRT2D().getTheta();
	double a = ellipse.getA();
	double b = ellipse.getB();
	double EfAxisRatio = b/a;

<<<<<<< HEAD

//************************* Bayes Fitting *************************/
	
/*
	int n = inWPtsAlongCols.cols();

	MatrixXd xVec2d = (inPts2dRotatedAlongCols.block(0,0,1,n)).transpose();
	MatrixXd yVec2d = (inPts2dRotatedAlongCols.block(1,0,1,n)).transpose();
	

	int maxPolyOrder = 6;

	
	MatrixXd x_1Vec = (inWPtsAlongCols.block(0,0,1,n)).transpose();	
	MatrixXd x_2Vec = (inWPtsAlongCols.block(1,0,1,n)).transpose();	
	MatrixXd x_3Vec = (inWPtsAlongCols.block(2,0,1,n)).transpose();	
	MatrixXd xVec = MatrixXd::Zero(n,1);	
	MatrixXd yVec = MatrixXd::Zero(n,1);

	
	for(size_t i = 0; i < n; i++) { 
		xVec(i,0) = x_1Vec(i,0)/x_3Vec(i,0);
		yVec(i,0) = x_2Vec(i,0)/x_3Vec(i,0);	
	}

	MatrixXd evidenceMat(maxPolyOrder+1,1);		
	MatrixXd AMetaLi = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);	
	MatrixXd AMetaEv = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);
	MatrixXd likelihoodMat(maxPolyOrder+1,1);
	Eigen::MatrixXd yVecHatMetaLi	= 
			MatrixXd::Zero(maxPolyOrder+1,n);
	
	//2d
	MatrixXd evidenceMat2d(maxPolyOrder+1,1);		
	MatrixXd AMetaLi2d = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);	
	MatrixXd AMetaEv2d = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);
	MatrixXd likelihoodMat2d(maxPolyOrder+1,1);
	Eigen::MatrixXd yVecHatMetaLi2d	= 
			MatrixXd::Zero(maxPolyOrder+1,n);

	cout << argv[1] << ": " << endl;
		for(size_t k = 0; k < maxPolyOrder; k++) {
			Eigen::MatrixXd Xmat = MatrixXd::Zero(k+1,n);
			Xmat = buildXMatIn(xVec, k);			
			Eigen::MatrixXd Xmat2d = MatrixXd::Zero(k+1,n);
			Xmat2d = buildXMatIn(xVec2d, k);

			vector<double> AvecLi = findALi(Xmat, yVec.transpose());	
			vector<double> AvecLi2d = findALi(Xmat2d, yVec2d.transpose());	
		
			for(size_t j = 0; j < AvecLi.size(); j++) {
				AMetaLi(k,j) = AvecLi[j];
				AMetaLi2d(k,j) = AvecLi2d[j];
			}
		
			Eigen::MatrixXd yVecHatLi = 
				predictY(Xmat, AMetaLi.block(k,0,1,k+1));
			Eigen::MatrixXd yVecHatLi2d = 
				predictY(Xmat2d, AMetaLi2d.block(k,0,1,k+1));
			
			yVecHatMetaLi2d.block(k,0,1,n) = yVecHatLi2d;

		
			evidenceMat(k,0) = 
				computePy_xvalpha(Xmat, yVec.transpose());
			likelihoodMat(k,0) = 
				computePy_xav(Xmat, yVec.transpose());
			evidenceMat2d(k,0) = 
				computePy_xvalpha(Xmat, yVec2d.transpose());
			likelihoodMat2d(k,0) = 
				computePy_xav(Xmat2d, yVec2d.transpose());


	MatrixXd outPtsAlongColsBays = MatrixXd::Zero(3,n);
*/
	/*
		for(size_t i = 0; i < n; i++) { 
			//outPtsAlongColsBays(0,i) = xVec(i,0) * x_3Vec(i,0);
			yVecHatMetaLi(k,i) = yVecHatLi(0,i) * x_3Vec(i,0);	
			//outPtsAlongColsBays(2,i) = x_3Vec(i,0);	
		}

		
		//cout<<outPtsAlongColsBays.block(0,0,1,n).transpose()<<endl;
		//cout<<outPtsAlongColsBays.block(1,0,1,n).transpose()<<endl;
		//cout<<outPtsAlongColsBays.block(2,0,1,n).transpose()<<endl;

	//printEigenMathematicaSci(evidenceMat, cout, "evidenceMat");	
	//printEigenMathematicaSci(outPtsAlongColsBays, cout, 		"outPtsAlongColsBays");

	}

	//printEigenMathematicaSci(yVecHatMetaLi, cout, "yVecHatMetaLi");
	//printEigenMathematicaSci(xVec, cout, "xVec");
	//cout << evidenceMat(maxPolyOrder-1,0) << endl;	

		
	//cout << "evidence for " << argv[1] <<":\n" << evidenceHom << endl;
	//cout << "likelihood for " << argv[1] <<":\n" << likelihoodMat(k,0) << endl;
*/
	
	/*********** End - Bayes Fitting ***********/

	//if(totalXYCost >= 0.0 && totalXYCost <=10) {
		

		//cout << argv[1] << ": " << endl;
		//MatrixXd jerk = simpleGradientAcrossCols(acceleration);

//cout << inWPtsAlongColsV3.cols() <<endl;
//cout << inWPtsAlongCols.cols() <<endl;
//		printEigenMathematica(inWPtsAlongCols, cout, "inPts");
	//	printEigenMathematica(e3D.getEIF().getPtsXYAlongCols(), cout, 			"inPtsRotated");
//		printEigenMathematica(pts2d, cout, "outPts2d");
  	//printEigenMathematica(homToCart(pts3d), cout, "outPts3d");
	//	printEigenMathematica(fixedThetas, cout, "Thetas");
	//	printEigenMathematica(speed, cout, "speed");
	//	printEigenMathematica(acceleration, cout, "acceleration");
	//	printEigenMathematica(jerk, cout, "jerk");
		

/*********** End - Ellipse Fitting ***********/


/*********** Terminal Output ***********/

/* Only reaches that pass the Ellipse Fit test */
if(totalXYCost >= 0.0) {

	/* Fitted Ellipse Parameters*/
  MatrixXd fixedThetas = fixThetas(thetas);
	MatrixXd speed = 
		simpleGradientAcrossCols(fixedThetas);
	MatrixXd acceleration = 
		simpleGradientAcrossCols(speed);

	cout << partId << ","
	     << reachCount  << ", " 
	     << reachCat << ", " 
	     << nRows << ", " 


	/*** The nominal plane in PCA ***/

		/* 3 Eigen Values */
	     << diagEigValsMat(0,0) << ", " 
			 << diagEigValsMat(1,1) << ", " 
			 << diagEigValsMat(2,2) << ", " 

		/* 3 Eigen Vectors v1,v2,v3 as regression pars*/
			 << EigVectsMatRowStacked(0,0) << ", " 
			 << EigVectsMatRowStacked(0,1) << ", " 
			 << EigVectsMatRowStacked(0,2) << ", " 
			 << EigVectsMatRowStacked(1,0) << ", " 
			 << EigVectsMatRowStacked(1,1) << ", " 
			 << EigVectsMatRowStacked(1,2) << ", " 
			 << EigVectsMatRowStacked(2,0) << ", " 
			 << EigVectsMatRowStacked(2,1) << ", " 
			 << EigVectsMatRowStacked(2,2) << ", " 


			 << planeStrike << ", " 
			 << planeDip << ", " 

			//for comarison
	 		 << planeAngleToXY << ", " 

	 		 << maxMinAngularErrs.first << ", " 
	 		 << maxMinAngularErrs.second << ", " 
	 		 << rake << ", " 

		/* Maximum Ellipsoid (error) shell h1,h2,h3-  */
			 << HypErrShellAxes(0,0) << ", " 
			 << HypErrShellAxes(1,0) << ", " 
			 << HypErrShellAxes(2,0) << ", " 

		//Fixed Length N-vector
			 <<	fixedLEcenterOffset << ", "
			 << fixedLEllipsoidAxes(0,0) << ", "
			 << fixedLEllipsoidAxes(1,0) << ", "
			 << fixedLEllipsoidAxes(2,0) << ", "


			 << alpha << ", " 
			 << totalXYCost << ", " 
			 << cX << ", " 
			 << cY << ", " 

			 << a << ", " 
			 << b << ", " 
			 << EfAxisRatio << endl;

/*
	printEigenMathematica 
		(inWPtsPCAaligned, cout, "inWPtsPCAaligned");
*/

	/*** The Cartesian error space of fitted 
			 orientation measurements ***/
/*
	MatrixXd x1ErrSpace2d = MatrixXd::Zero(nRows,2);
	x1ErrSpace2d.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,0,nRows,1);
	x1ErrSpace2d.block(0,1,nRows,1) = 
		HypErrBounds.block(0,0,nRows,1);

	MatrixXd minusX1ErrSpace2d = MatrixXd::Zero(nRows,2);
	minusX1ErrSpace2d.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,0,nRows,1);
	minusX1ErrSpace2d.block(0,1,nRows,1) = 
		- HypErrBounds.block(0,0,nRows,1);

	MatrixXd x2ErrSpace2d = MatrixXd::Zero(nRows,2);
	x2ErrSpace2d.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,1,nRows,1);
	x2ErrSpace2d.block(0,1,nRows,1) = 
		HypErrBounds.block(0,1,nRows,1);
	
	MatrixXd minusX2ErrSpace2d = MatrixXd::Zero(nRows,2);
	minusX2ErrSpace2d.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,1,nRows,1);
	minusX2ErrSpace2d.block(0,1,nRows,1) = 
		- HypErrBounds.block(0,1,nRows,1);

	printEigenMathematica
		(x1ErrSpace2d, cout, "x1ErrSpace2d");
	printEigenMathematica
		(minusX1ErrSpace2d, cout, "minusX1ErrSpace2d");
	printEigenMathematica
		(x2ErrSpace2d, cout, "x2ErrSpace2d");
	printEigenMathematica
		(minusX2ErrSpace2d, cout, "minusX2ErrSpace2d");
*/	
	/******************/

	/*** Data Variance - The data making up the
 			 planar measuremen***/
/*	
	MatrixXd in2dx13PtsPCAaligned = 
		MatrixXd::Zero(nRows,2);
	in2dx13PtsPCAaligned.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,0,nRows,1);
	in2dx13PtsPCAaligned.block(0,1,nRows,1) = 
		inWPtsPCAaligned.block(0,2,nRows,1);

	MatrixXd in2dx23PtsPCAaligned = 
		MatrixXd::Zero(nRows,2);
	in2dx23PtsPCAaligned.block(0,0,nRows,1) = 
		inWPtsPCAaligned.block(0,1,nRows,1);
	in2dx23PtsPCAaligned.block(0,1,nRows,1) = 
		inWPtsPCAaligned.block(0,2,nRows,1);

	printEigenMathematica
		(in2dx13PtsPCAaligned, cout, "in2dx13PtsPCAaligned");
	printEigenMathematica
		(in2dx23PtsPCAaligned, cout, "in2dx23PtsPCAaligned");
*/
	/******************/

	for(size_t i = 0; i < nRows-2; i++) { 
	/*
		 	cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
					 << i+1 << ","
					 << "1, " 
					 << inWPtsAlongRows(i,0) << ", "
					 << "1, " 
					 << inWPtsAlongRows(i,1) << ", " 
					 << "1, " 
					 << inWPtsAlongRows(i,2) << ", "
					 << fixedThetas(0,i) << ", "
					 << speed(0,i) << ", "
					 << acceleration(0,i) << endl;


			cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
					 << i+1 << ","
					 << "2, " 
					 << homToCart(pts3d)(0,i) << ", "
					 << "2, " 
					 << homToCart(pts3d)(1,i) << ", " 
					 << "2, " 
					 << homToCart(pts3d)(2,i) << endl;
*/
/*
			cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
					 << inWPtsPCAaligned(i,0) << ", "
					 << "1, " 
					 << inWPtsPCAaligned(i,2) << ", "
					 << inWPtsPCAaligned(i,1) << ", "
			     << "1, " 
					 << inWPtsPCAaligned(i,2) << endl;
				
			cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
					 << inWPtsPCAaligned(i,0) << ", "
					 << "2, " << HypErrBounds(i,0) << ", " 
					 << inWPtsPCAaligned(i,1) << ", "
					 << "2, " << HypErrBounds(i,1) << endl;
				
			cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
					 << inWPtsPCAaligned(i,0) << ", "
					 << "3, " << -HypErrBounds(i,0) << ", " 
					 << inWPtsPCAaligned(i,1) << ", "
					 << "3, " << -HypErrBounds(i,1) << endl;
*/

			}
/*
		double sliceSize = (M_PI * 2.0)/double(sliceN);

		for(size_t i = 0; i < pts3dSim.cols(); i++) { 
			double theta = double (i) * sliceSize;
		 	
			cout << partId << ","
	     	 	 << reachCount  << ", " 
	     	 	 << reachCat << ", " 
	     	 	 << i << ", " 
	     	 	 << theta << ", " 
					 << homToCart(pts3dSim)(0,i) << ", "
					 << homToCart(pts3dSim)(1,i) << ", " 
					 << homToCart(pts3dSim)(2,i) << endl;
		}
*/
	}
=======
	//printEigenMathematica(e3D.getEIF().getPtsXYAlongCols(), cout, "inPtsRotated");
	//cout << "Ellipse-Fit-Costs:\n" << inPtsRotated << endl;
	
	//cout << "cX:" << cX << endl;
	//cout << "cY:" << cY << endl;
	//cout << "alpha:" << alpha << endl;
	//cout << "a:" << a << endl;
	//cout << "b:" << b << endl;	
	if(totalXYCost >= 0.0) {
		//cout << eigenSys.first(2,0)<< endl;
		//cout << /*"Axis-Ratio:" <<*/ EfAxisRatio << endl;	
		//cout << /*"Ellipse-Total-Fit-Cost:" <<*/ totalXYCost << endl;
		//cout << "Ellipse-Fit-Costs:\n" << fitCosts << endl;
	
	

	/*********** Bayes Fitting ***********/

		int n = inPts2dRotatedAlongCols.cols();
		MatrixXd xVec = (inPts2dRotatedAlongCols.block(0,0,1,n)).transpose();
		MatrixXd yVec = (inPts2dRotatedAlongCols.block(1,0,1,n)).transpose();	
		int maxPolyOrder = 5;
		MatrixXd evidenceMat(maxPolyOrder+1,1);		
		MatrixXd AMetaLi = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);	
		MatrixXd AMetaEv = MatrixXd::Zero(maxPolyOrder+1,maxPolyOrder+1);
		MatrixXd likelihoodMat(maxPolyOrder+1,1);
		Eigen::MatrixXd yVecHatMetaLi	= 
			MatrixXd::Zero(maxPolyOrder+1,n);
		Eigen::MatrixXd yVecHatMetaEv	= 
			MatrixXd::Zero(maxPolyOrder+1,n);
size_t k = 4;
		//for(size_t k = 0; k < maxPolyOrder; k++) {
			Eigen::MatrixXd Xmat = MatrixXd::Zero(k+1,n);
			Xmat = buildXMatIn(xVec, k);
			
			vector<double> AvecLi = findALi(Xmat, yVec.transpose());	
		
			for(size_t j = 0; j < AvecLi.size(); j++) 
				AMetaLi(k,j) = AvecLi[j];

		
		Eigen::MatrixXd yVecHatLi = 
				predictY(Xmat, AMetaLi.block(k,0,1,k+1));
		yVecHatMetaLi.block(k,0,1,n) = yVecHatLi;

		
		evidenceMat(k,0) = 
				computePy_xvalpha(Xmat, yVec.transpose());
		likelihoodMat(k,0) = 
				computePy_xav(Xmat, yVec.transpose());
	//}

	
	//cout << evidenceMat(k,0) << endl;	
	cout << /*"likelihood for " << argv[1] <<":\n" <<*/ likelihoodMat(k,0) << endl;

	/*********** End - Bayes Fitting ***********/

/*
	MatrixXd fixedThetas = fixThetas(thetas);
	MatrixXd speed = simpleGradientAcrossCols(fixedThetas);
	MatrixXd acceleration = simpleGradientAcrossCols(speed);
	MatrixXd jerk = simpleGradientAcrossCols(acceleration);*/


	/*printEigenMathematica(inWPtsAlongCols, cout, "inPts");
	printEigenMathematica(e3D.getEIF().getPtsXYAlongCols(), cout, "inPtsRotated");
	printEigenMathematica(pts2d, cout, "outPts2d");
	printEigenMathematica(pts3d, cout, "outPts3d");
	printEigenMathematica(fixedThetas, cout, "Thetas");
	printEigenMathematica(speed, cout, "speed");
	printEigenMathematica(acceleration, cout, "acceleration");
	printEigenMathematica(jerk, cout, "jerk");*/

	/*********** End - Ellipse Fitting ***********/

	
	return 0;
}
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
	return 0;
}
