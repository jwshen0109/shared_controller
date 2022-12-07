#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Core>

#include <vector>

using namespace Eigen;
using namespace std;

Eigen::VectorXd polyfit(vector<double>& xvals, vector<double>& yvals, int order){

    assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::VectorXd yvals_tmp = Eigen::VectorXd::Zero(yvals.size());
    for(int i=0; i<yvals.size(); i++){
        yvals_tmp(i) = yvals[i];
    }
	Eigen::MatrixXd A(xvals.size(), order + 1);
	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}
	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals[j];
		}
	}
	auto Q = A.householderQr();
	auto result = Q.solve(yvals_tmp);
	return result;

}

int main(){
    Matrix
}