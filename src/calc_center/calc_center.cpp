// basic file operations
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
using namespace std;
using namespace Eigen;
#define NUM_ROWS 18
MatrixXf points(NUM_ROWS,3);

MatrixXf cartToHom(MatrixXf pointsC) {
    MatrixXf pointsH(pointsC.rows() + 1, pointsC.cols());
    for(size_t row = 0; row < pointsC.rows(); row++) {
        for(size_t col = 0; col < pointsC.cols(); col++) {
            pointsH(row, col) = pointsC(row, col);
        }
    }
    for(size_t col = 0; col < pointsC.cols(); col++) {
        pointsH(pointsC.rows(), col) = 1.0f;
    }
    return pointsH;
}

void printMathematica(MatrixXf A) {
    cout << "{";
    for(size_t row = 0; row < A.rows(); row++) {
        cout << "{";
        for(size_t col = 0; col < (A.cols() - 1); col++) {
            cout << A(row, col) << ",";
        }
        cout << "}";
    }
    cout << "}";
}


int main(int argc, char **argv) {
    std::ifstream t(argv[1]);
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    //cout << str << endl;

    char cstr[str.length() + 1];
    cstr[str.length()] = 0;
    memcpy(cstr, str.c_str(), str.length());
    //cout << cstr << endl;
    char *token = strtok(cstr, ",");
    int rowCtr = 0;
    int colCtr = 0;
    while(token != NULL) {
        //cout << token << endl;
        if((rowCtr < NUM_ROWS) && (colCtr < 3)) {
            points(rowCtr, colCtr) = strtof(token, NULL);
        }
        //cout << token << "  " << colCtr << endl;
        colCtr++;
        if(colCtr == 3) {
            colCtr = 0;
            rowCtr++;
        }
        token = strtok(NULL, ",");
    }

    MatrixXf pointsH = cartToHom(points.transpose());
    cout <<"pointsH"<<endl<< pointsH <<endl;
    JacobiSVD<MatrixXf> svd(pointsH.transpose(), ComputeThinV);
    MatrixXf vMat = svd.matrixV();
    cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << vMat << endl;
    Quaternion<float> quad(vMat(3,3), vMat(0,3), vMat(1,3), vMat(2,3));
    quad.normalize();
    MatrixXf rotMat = quad.toRotationMatrix();
    cout << "rotMat" <<endl << rotMat<<endl;
    cout <<"points"<<endl<<points<<endl;
    MatrixXf rotatedPoints =  rotMat * points.transpose();
    cout <<"rotated points" <<endl<<rotatedPoints <<endl;
    for(int col = 0; col < rotatedPoints.cols(); col++){
        rotatedPoints(2, col)= 1.0;
    }
    cout << "2D:"<<endl<< rotatedPoints <<endl;
    MatrixXf A = rotatedPoints.transpose();
    cout << "A:"<<endl<< A <<endl;
    JacobiSVD<MatrixXf> svd2(A, ComputeFullU | ComputeFullV);
    VectorXf b(rotatedPoints.cols());
    for(int r =0; r <rotatedPoints.cols(); r++){
        b(r) = pow(rotatedPoints(0,r),2) +pow(rotatedPoints(1,r),2);
   }
   
    cout << "b" <<endl << b<<endl;
    cout <<"?" <<endl<< svd2.solve(b) <<endl;
}