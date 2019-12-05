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
        cout << A(row, A.cols() - 1) << "},";
    }
    cout << "}" <<endl;
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
    //put coordinates in the homogeneous coordinate system
    MatrixXf pointsH = cartToHom(points.transpose());
    cout <<"pointsH"<<endl<< pointsH <<endl;
    //we do singular value decomposition to find the v matrix
    JacobiSVD<MatrixXf> svd(pointsH.transpose(), ComputeThinV);
    MatrixXf vMat = svd.matrixV();
    cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << vMat << endl;
    //the very last column describes the rotation to fit coordinates to plane
    Vector3f newCoord(vMat(0,3), vMat(1,3), vMat(2,3));
    Vector3f oldCoord(0.0,0.0,1.0);
    Quaternion<float> quad = Quaternion<float>::FromTwoVectors(newCoord,oldCoord);
    quad.normalize();
    //we transform the quaterion into a rotation matrix so we can easily multiply it by the points
    Matrix3f rotMat = quad.toRotationMatrix();
    cout << "rotMat" <<endl << rotMat<<endl;
    cout <<"points"<<endl<<points<<endl;
    //we transpose all the points onto the plane
    MatrixXf rotatedPoints =  rotMat * points.transpose();
    cout <<"rotated points" <<endl<<rotatedPoints <<endl;
    //printMathematica(rotatedPoints);
    float zCenter = rotatedPoints(2,0);
    for(int col = 0; col < rotatedPoints.cols(); col++){
        rotatedPoints(2, col)= 1.0;
    }
    cout << "2D:"<<endl<< rotatedPoints <<endl;
    /*Solution by method of least squares:
    A*c = b, c' = argmin(||A*c - b||^2)
    A = [x y 1], b = [x^2+y^2] */
    MatrixXf A = rotatedPoints.transpose();
    cout << "A:"<<endl<< A <<endl;
    JacobiSVD<MatrixXf> svdCenter(A, ComputeFullV|ComputeFullU);
    //now calculate the 
    VectorXf b(A.rows());
    for (int i = 0; i < A.rows(); i++){
        // b = X^2+y^2
        b(i)= A(i,0)*A(i,0)+A(i,1)*A(i,1);
    }
    Vector3f center = svdCenter.solve(b);
    cout <<"soln"<<endl<< center  <<endl;
    float xCenter = center(0)/2;
    float yCenter = center(1)/2;
    cout << "X: " << xCenter << endl <<"Y: "<<yCenter<<endl <<"Z"<<zCenter<<endl;
    Vector3f centerMat(xCenter,yCenter,zCenter);
    cout <<"centerMat on plane: "<<centerMat<<endl<<"in our coords: " <<rotMat.transpose() * centerMat<<endl;
    
    // now we save the center and the rotation matrix

    /*//find matrix that describes equations of ellipses
    //𝑎𝑥2+𝑏𝑥𝑦+𝑐𝑦2+𝑑𝑥+𝑒𝑦+𝑓=0
    MatrixXf ellipse(A.rows(), 6);
    for(int r = 0; r < ellipse.rows();r++){
        double x = A(r,0);
        double y = A(r,1);
        ellipse(r,0)=x*x;
        ellipse(r,1)=x*y;
        ellipse(r,2)=y*y;
        ellipse(r,3)=x;
        ellipse(r,4)=y;
        ellipse(r,5)=1;
    }
    cout <<"ellipses equations matrix" <<endl<< ellipse <<endl;
    JacobiSVD<MatrixXf> svdEllipse(ellipse, ComputeFullV | ComputeFullU);
    MatrixXf vMatEllipse = svdEllipse.matrixV();
    VectorXf ellipse_soln(vMatEllipse.rows());
    double a = vMatEllipse(0,5);
    double b = vMatEllipse(1,5);
    double c = vMatEllipse(2,5);
    double d = vMatEllipse(3,5);
    double e = vMatEllipse(4,5);
    double f = vMatEllipse(5,5);
    cout <<"ellipse soln"<<endl<<ellipse_soln<<endl;
    Matrix3f quad_equation;
    quad_equation << a, b/2, d/2,
                    b/2, c, e/2,
                    d/2, e/2, f;
    printMathematica(quad_equation);*/
    

}