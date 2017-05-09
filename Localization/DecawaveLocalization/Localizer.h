#ifndef LOCALIZER_H
#define LOCALIZER_H
#include <vector>
#include <array>
#include <iostream>
#include <math.h>

using namespace std;
class Localizer
{
public:
	Localizer(vector<vector<float>>receivers, float height);
	~Localizer();
	vector<vector<float>> receivers;	// xyz position of all receivers
	int numReceivers = 0;
	float R2_z = 0.0;					// fixed, known z position of R2 Decawave


	vector<float> localize(const vector<float>& x0, const vector<float>& dist);
	//void computeResiduals(const vector<float>& x, const vector<float>& dist, vector<float>& residuals);
	//void computeJacobian(const vector<float>& x, const vector<float>& dist, vector<vector<float>>& jacobian);

private:
	// Gauss Newton Tuning Parameters
	float c = (float).00001;
	float alpha = (float).5;
	float tol_df = (float)pow(10.0, -15.0);
	float tol_dx = (float)pow(10.0, -15.0);
	float tol_f = (float)pow(10.0, -15.0);
	int maxIter = 20;

	float norm(const vector<float>& vec);
	float distance(const vector<float>& vec1, const vector<float>& vec2);
	float dot(const vector<float>& vec1, const vector<float>& vec2);
};

#endif