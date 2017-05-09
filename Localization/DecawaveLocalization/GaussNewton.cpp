#include "GaussNewton.h"


GaussNewton::GaussNewton()
{
}


GaussNewton::~GaussNewton()
{
}

vector<float> GaussNewton::optimize() {
	vector<float> residuals(numFunctions,0);
	vector<vector<float>> jacobian(numFunctions, vector<float>(numVars,0));
	vector<float> x(x0);
	vector<float> grad(numVars,0);

	// initialize residuals, jacobian, gradient, value of objective function
	computeResiduals(x0, obs, residuals);
	computeJacobian(x0, obs, jacobian);
	computeGradient(jacobian, residuals, grad);
	float obj_f = computeObjectiveFunction(residuals);

	// initialize variables to keep track of tolerances
	// make sure tolerance checks fail on first iteration
	float prev_obj_f = obj_f + 5 * tol_f;
	vector<float>prev_x(x0);
	prev_x[0] += 100.0;	// to make sure tolerance check fails
	int iter = 0;

	// initialize descent direction
	vector<float>dGN(numVars, 0);

	while (norm(grad) > tol_df && distance(prev_x, x) > tol_dx &&
		fabs(prev_obj_f-obj_f) > tol_f && iter < maxIter || iter == 0) {

		// Solve for descent direction
		getDescentDirection(jacobian, residuals, dGN);

		// Update prev_x, prev_obj_f
		prev_x = x;
		prev_obj_f = obj_f;

		// Armijo backtracking
		float step = initialStepSize/alpha;
		int armIter = 0;

		do {
			// maximum backtracking iterations reached, objective function cannot be reduced
			// return whatever x currently is
			if (armIter >= maxArmIter) {
				return x;
			}

			// reduce the step size by factor of alpha
			step = alpha*step;

			// Compute x + step*dGN
			for (int j = 0; j < numVars; j++) {
				x[j] = prev_x[j] + step*dGN[j];
			}
			
			// update the residuals, value of objective function
			computeResiduals(x, obs, residuals);
			obj_f = computeObjectiveFunction(residuals);

			// increment iteration counter
			armIter += 1;
		} while (obj_f > prev_obj_f + c*step*dot(grad, dGN));	// check Wolfe condition

		// update Jacobian and gradient
		computeJacobian(x, obs, jacobian);
		computeGradient(jacobian, residuals, grad);

		iter++;
	}

	return x;
}

// compute the 2-norm of a vector
float GaussNewton::norm(const vector<float>& vec) {
	float temp = 0.0;
	for (int j = 0; j < vec.size(); j++) {
		temp += pow(vec[j], 2);
	}
	return sqrtf(temp);
}

// compute the Euclidean distance between two vetors
float GaussNewton::distance(const vector<float>& vec1, const vector<float>& vec2) {
	float temp = 0.0;
	for (int j = 0; j < vec1.size(); j++) {
		temp += pow(vec1[j] - vec2[j], 2);
	}
	return sqrtf(temp);
}

// compute the dot product between two vectors
float GaussNewton::dot(const vector<float>& vec1, const vector<float>& vec2) {
	float ans = 0.0;

	for (int j = 0; j < vec1.size(); j++) {
		ans += vec1[j] * vec2[j];
	}

	return ans;
}

// compute the gradient for objective function of the form .5*(norm(r)^2). g = transpose(J)*r.
void GaussNewton::computeGradient(const vector<vector<float>>&jacobian, const vector<float>&residuals, vector<float>& grad) {
	for (int c = 0; c < numVars; c++) {
		grad[c] = 0;
		for (int r = 0; r < numFunctions; r++) {
			grad[c] += jacobian[r][c] * residuals[r];
		}
	}
}

// compute the objective function, .5*(norm(r)^2)
float GaussNewton::computeObjectiveFunction(const vector<float>& residuals) {
	return (float).5*pow(norm(residuals), 2);
}

// Obtain the Gauss Newton descent direction, by solving J*dGN = -r.
// dGN = descent direction Gauss Newton
void GaussNewton::getDescentDirection(const vector<vector<float>>&jacobian, const vector<float>&residuals, vector<float>& dGN) {
	Eigen::MatrixXf J(numFunctions, numVars);
	Eigen::VectorXf res(numFunctions);
	for (int r = 0; r < numFunctions; r++) {
		for (int c = 0; c < numVars; c++) {
			J(r, c) = jacobian[r][c];
		}
	}

	for (int r = 0; r < numFunctions; r++) {
		res[r] = -residuals[r];
	}

	VectorXf ans = J.colPivHouseholderQr().solve(res);

	for (int c = 0; c < numVars; c++) {
		dGN[c] = ans(c);
	}

}