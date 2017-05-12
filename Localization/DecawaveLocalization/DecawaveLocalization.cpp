#include <Windows.h>
#include <conio.h>

#include <chrono>
#include <iostream>
#include <vector>

#include "SerialPort.h"

#include "Localizer.h"
#include "GaussNewton.h"

using std::vector;
using std::string;

using namespace std::chrono;

#define BUFFER_SIZE 4096
constexpr int NUM_RECEIVERS = 3;
constexpr float r2z = 0.8f;
constexpr float receiverz = 1.8288f;
constexpr float calibration_offset = -0.7f;

void setv(vector<float>& v, float x, float y, float z) {
	v[0] = x;
	v[1] = y;
	v[2] = z;
}

void generateGraph(vector<vector<float>>& dists, vector<vector<float>>& receivers) {
	// TODO work with more than 3 receivers
	// Average values in the receiver graph
	for (int i = 0; i < NUM_RECEIVERS; i++) {
		for (int j = i + 1; j < NUM_RECEIVERS; j++) {
			float average = (dists[i][j] + dists[j][i]) * 0.5f;
			dists[i][j] = average;
			dists[j][i] = average;
		}
	}
	
	// Receiver positions
	vector<Eigen::Vector3f> positions(3);
	// Assume the first receiver is at a certain point
	Eigen::Vector3f origin(0, 0, receiverz);
	positions[0] = origin;

	// Assume the second tag is in a direction
	Eigen::Vector3f direction(7.8034f, 4.8756f, 0);
	direction.normalize();
	positions[1] = positions[0] + direction * dists[0][1];

	// Assume the third tag is in the plane in the direction of the given vector
	Eigen::Vector3f plane_normal(1.f, -1.f, 0.f);
	plane_normal.normalize();
	float semiperimeter = (dists[0][1] + dists[1][2] + dists[0][2]) * 0.5f;
	float area = sqrt(semiperimeter * (semiperimeter - dists[0][1])
		* (semiperimeter - dists[0][2]) * (semiperimeter - dists[1][2]));
	float height = 2.f * area / dists[0][1];
	float length = sqrt(dists[0][2] * dists[0][2] - height * height);
	Eigen::Vector3f vec01 = positions[1] - positions[0];
	vec01.normalize();
	Eigen::Vector3f orthog = vec01 * (plane_normal.dot(vec01));
	Eigen::Vector3f diff = plane_normal - orthog;
	diff.normalize();
	float t = length / dists[0][1];
	positions[2] = (1.f - t) * positions[0] + t * positions[1] + diff * height;

	// Convert eigen vector3f to vectors
	for (int i = 0; i < NUM_RECEIVERS; i++) {
		receivers[i][0] = positions[i].x();
		receivers[i][1] = positions[i].y();
		receivers[i][2] = positions[i].z();
	}
}

int main(int argc, char ** argv)
{
	if (argc < 2) {
		//printf("Usage: %s <COM port>\n", argv[0]);
		//return 0;
		argv[1] = "\\\\.\\COM15";
	}

	SerialPort conn(argv[1], CBR_115200);

	vector<vector<float>> receivers;
	for (int i = 0; i < NUM_RECEIVERS; i++) {
		receivers.push_back(vector<float>{0, 0, 0});
	}
	vector<float> dist(NUM_RECEIVERS, 0);
	vector<float> x(NUM_RECEIVERS, 0);
	vector<vector<float>> receiver_graph;
	for (int i = 0; i < NUM_RECEIVERS; i++) {
		receiver_graph.push_back(vector<float>(NUM_RECEIVERS, 0));
	}

	char mode = 'p';

	Localizer local(receivers, 0);
	char buffer[BUFFER_SIZE + 1];
	string data = "";

	char modes[] = { 'Q', 'P', 'C', 'N', 'L', VK_ESCAPE };
	auto lastkp = std::chrono::steady_clock::now();

	while (mode != VK_ESCAPE) {
		auto ctime = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(ctime - lastkp).count() > 250) {
			for (int i = 0; i < sizeof(modes); i++) {
				if ((GetAsyncKeyState(modes[i]) & 0x8000) != 0) {
					mode = modes[i];
					printf("Switched to mode %c\n", mode);
					lastkp = ctime;
				}
			}
		}

		if (conn.isConnected()) {
			int read = conn.read(buffer, BUFFER_SIZE);
			if (read > 0) {
				data = data + string(buffer, read);
			}
		}

		switch (mode) {
		case 'Q': // Query
		case 'P': // Poll
		case 'C': // Send polls
		case 'N': // Send queries
			buffer[0] = mode;
			conn.write(buffer, 1);
			mode = ' ';
			break;
		case 'L': // Localize
			generateGraph(receiver_graph, receivers);
			local = Localizer(receivers, r2z);
			mode = ' ';
			break;
		default:
			break;
		}

		// Parse data
		size_t index;
		while ((index = data.find("\n")) != string::npos) {
			string result = data.substr(0, index);
			printf("%s\t", result.c_str());
			if (result[0] == 'P') { // Poll result
				int tag;
				float distance;
				int read = sscanf_s(result.c_str(), "P,%d,%f", &tag, &distance);
				if (read == 2) {
					tag--;
					distance += calibration_offset;
				}
				dist[tag] = distance;
				high_resolution_clock::time_point t1 = high_resolution_clock::now();
				x = local.localize(x, dist);
				high_resolution_clock::time_point t2 = high_resolution_clock::now();
				auto duration = duration_cast<microseconds>(t2 - t1).count();
				printf("% 3.3f    % 3.3f    % 3.3f   %lld\n", x[0], x[1], x[2], duration);
			}
			else if (result[0] == 'Q') { // Query distance result
				int tagFrom, tagTo, samples;
				float total_distance;
				int read = sscanf_s(result.c_str(), "Q,%d,%d,%d,%f", &tagFrom, &tagTo, &samples, &total_distance);
				if (read == 4 && samples > 0) {
					tagFrom--;
					tagTo--;
					receiver_graph[tagFrom][tagTo] = total_distance / (float)samples + calibration_offset;
				}
				printf("\n");
			}
			else {
				printf("\n");
			}
			data.erase(0, index + 1);
		}
	}
	return 0;
}
