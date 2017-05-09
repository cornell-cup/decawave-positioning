#ifndef GAUSSNEWTON_H
#define GAUSSNEWTON_H

#include <math.h>
#include <vector>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

using namespace std;
using namespace Eigen;

class GaussNewton
{
public:
	GaussNewton();
	~GaussNewton();

	vector<float> optimize();

	// Required variables for optimization
	int numVars = 0;
	int numFunctions = 0;
	vector<float> x0;
	vector<float> obs;

	// residual/jacobian functions should take the parameters x (current state),
	// obs (observations) and the vector in which the result will be stored
	//void(*computeResiduals)(const vector<float>&, const vector<float>&, vector<float>&);
	//void(*computeJacobian) (const vector<float>&, const vector<float>&, vector<vector<float>>&);
	function<void(const vector<float>&, const vector<float>&, vector<float>&)> computeResiduals;
	function<void(const vector<float>&, const vector<float>&, vector<vector<float>>& )> computeJacobian;

	// Gauss Newton Tuning Parameters
	float c = (float).00001;
	float alpha = (float).5;
	float initialStepSize = 2.0;
	float tol_df = (float)pow(10.0, -15.0);
	float tol_dx = (float)pow(10.0, -15.0);
	float tol_f = (float)pow(10.0, -15.0);
	int maxIter = 50;
	int maxArmIter = 50;

private:
	void computeGradient(const vector<vector<float>>& jacobian, const vector<float>& residuals, vector<float>& grad);
	float computeObjectiveFunction(const vector<float>& residuals);
	void getDescentDirection(const vector<vector<float>>&jacobian, const vector<float>&residuals, vector<float>& dGN);
	float norm(const vector<float>& vec);
	float distance(const vector<float>& vec1, const vector<float>& vec2);
	float dot(const vector<float>& vec1, const vector<float>& vec2);
};

#endif