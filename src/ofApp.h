#pragma once

#include "ofMain.h"
#include "ofxImGui.h"

#include <vector>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

static const float kFPS = 60.0f;

struct PeseudoRandom {
	virtual ~PeseudoRandom() {}

	virtual uint32_t generate() = 0;
	virtual double uniform() = 0;
	virtual double uniform(double a, double b) = 0;
};
struct Xor : public PeseudoRandom {
	Xor() {

	}
	Xor(uint32_t seed) {
		_y = std::max(seed, 1u);
	}

	// 0 <= x <= 0x7FFFFFFF
	uint32_t generate() {
		_y = _y ^ (_y << 13); _y = _y ^ (_y >> 17);
		uint32_t value = _y = _y ^ (_y << 5); // 1 ~ 0xFFFFFFFF(4294967295
		return value >> 1;
	}
	// 0.0 <= x < 1.0
	double uniform() {
		return double(generate()) / double(0x80000000);
	}
	double uniform(double a, double b) {
		return a + (b - a) * double(uniform());
	}
public:
	uint32_t _y = 2463534242;
};

inline glm::vec3 uniform_on_unit_sphere(PeseudoRandom *random) {
	glm::vec3 d;
	float sq = 0.0f;
	do {
		d.x = random->uniform(-1.0, 1.0);
		d.y = random->uniform(-1.0, 1.0);
		d.z = random->uniform(-1.0, 1.0);

		sq = glm::length2(d);
	} while (sq < 0.0001f || 1.0f < sq);
	d /= glm::sqrt(sq);
	return d;
}

struct DistanceConstraint {
	int index0;
	int index1;
	float length = 0.0;
};

struct FloorConstraint {
	float h = 0.0f;
};

class ofApp : public ofBaseApp{
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void initialize();

	ofxImGui::Gui _imgui;
	ofEasyCam _camera;

	std::vector<glm::vec3> _points;
	std::vector<float> _mass;
	std::vector<glm::vec3> _pointsVelocity;
	std::vector<glm::vec3> _pointsMoved;

	std::vector<DistanceConstraint> _dconstraints;
	FloorConstraint _floorConstraint;

	Xor _random;

	float _stiffness = 1.0f;
};
