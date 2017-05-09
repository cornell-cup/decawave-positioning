#include "Localizer.h"
#include "GaussNewton.h"
#include <functional>

using namespace std;
Localizer::Localizer(vector<vector<float>>receivers, float R2_z)
{
	this->receivers = receivers;
	numReceivers = (int)receivers.size();
	this->R2_z = R2_z;
}


Localizer::~Localizer()
{
}

vector<float> Localizer::localize(const vector<float>& x0, const vector<float>& dist) {
	if (dist.size() != numReceivers) {
		cout << "Error, need the same number of receivers as measurements.\n";
		return vector<float>();
	}

	// Initialize optimizer, use default computation tolerances
	GaussNewton optimizer;
	optimizer.numVars = 3;
	optimizer.numFunctions = 4;
	optimizer.computeResiduals = [this](const vector<float>& x, const vector<float>& dist, vector<float>& residuals) -> void{
		for (int j = 0; j < dist.size(); j++) {
			residuals[j] = this->distance(this->receivers[j], x) - dist[j];
		}
		residuals[dist.size()] = x[2] - this->R2_z;
	};
	//optimizer.computeResiduals = //&Localizer::computeResiduals;
	optimizer.computeJacobian = [this](const vector<float>& x, const vector<float>& dist, vector<vector<float>>& jacobian) -> void{
		for (int r = 0; r < jacobian.size()-1; r++) {
			vector<float>diff(jacobian[0].size(), 0);
			for (int c = 0; c < jacobian[0].size(); c++) {
				diff[c] = x[c] - receivers[r][c];
			}
			float diffNorm = this->norm(diff);

			for (int c = 0; c < jacobian[0].size(); c++) {
				jacobian[r][c] = diff[c] / diffNorm;
			}
		}

		jacobian[jacobian.size() - 1][0] = 0;
		jacobian[jacobian.size() - 1][1] = 0;
		jacobian[jacobian.size() - 1][2] = 1;
	};
	//&Localizer::computeJacobian;

	optimizer.x0 = x0;
	optimizer.obs = dist;

	return optimizer.optimize();
}

float Localizer::norm(const vector<float>& vec) {
	float temp = 0.0;
	for (int j = 0; j < vec.size(); j++) {
		temp += pow(vec[j], 2);
	}
	return sqrtf(temp);
}

float Localizer::distance(const vector<float>& vec1, const vector<float>& vec2) {
	float temp = 0.0;
	for (int j = 0; j < vec1.size(); j++) {
		temp += pow(vec1[j] - vec2[j], 2);
	}
	return sqrtf(temp);
}

float Localizer::dot(const vector<float>& vec1, const vector<float>& vec2) {
	float ans = 0.0;

	for (int j = 0; j < vec1.size(); j++) {
		ans += vec1[j] * vec2[j];
	}

	return ans;
}

