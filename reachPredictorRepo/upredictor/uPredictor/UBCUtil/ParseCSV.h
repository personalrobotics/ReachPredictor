#ifndef PARSE_CSV_H
#define PARSE_CSV_H

#include <Eigen/Core>
#include "csv.h"
#include <string>

using namespace Eigen;
using namespace std;

string getWholeFile(string fileName);
<<<<<<< HEAD
string getCharfromString(string fileName);
string getPartID(string fileName);

=======
>>>>>>> e070f2e51ee0c62744927929a7a1819317e943b7
int countCols(string inLine);
int countRowsCSV(string inMat, string inCol);
int countRowsFColCSV(string inMat); 
void fillMatCSV(int inColNo, string inLine, string inX,
	 string inY, string inZ, MatrixXd &mat);
void fillMatWholeCSV(int inColNo, string inLine, MatrixXd &mat);
MatrixXd parseCSV(string inMat);

#endif
