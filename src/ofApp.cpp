#include "ofApp.h"

inline ofVec3f toOf(glm::vec3 p) {
	return ofVec3f(p.x, p.y, p.z);
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetVerticalSync(false);
	ofSetFrameRate(kFPS);

	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	initialize();
}

void ofApp::initialize() {
	_points.clear();
	_dconstraints.clear();
	_mass.clear();
	_pointsVelocity.clear();
	_pointsMoved.clear();

	glm::vec3 o(0.0, 2.0, 0.0);
	_points.push_back(o);

	float constraintLength = 0.1f;
	for (int i = 0; i < 20; ++i) {
		glm::vec3 d = uniform_on_unit_sphere(&_random);

		d.x *= 2.0f;
		d = glm::normalize(d);

		glm::vec3 newP = o + d * constraintLength;
		_points.push_back(newP);

		DistanceConstraint dc;
		dc.index0 = i;
		dc.index1 = i + 1;
		dc.length = constraintLength;
		_dconstraints.push_back(dc);

		o = newP;
	}

	// cube
	{
		float s = 0.5f;
		glm::vec3 ps[8] = {
			{ -s, -s, -s },
			{  s, -s, -s },
			{  s, -s,  s },
			{ -s, -s,  s },

			{ -s, s, -s },
			{ s,  s, -s },
			{ s,  s,  s },
			{ -s, s,  s },
		};

		auto q = glm::rotation(glm::vec3(0.0, 1.0, 0.0), glm::normalize(glm::vec3(0.1, 0.7, 0.3)));

		std::vector<int> indices;
		for (int i = 0; i < 8; ++i) {
			indices.push_back(_points.size());
			_points.push_back( q * ps[i] + glm::vec3(2.0f, 2.0f, 0.0f));
		}
		
		auto line = [&](int a, int b) {
			DistanceConstraint dc;
			dc.index0 = indices[a];
			dc.index1 = indices[b];
			dc.length = glm::distance(_points[dc.index0], _points[dc.index1]);
			_dconstraints.push_back(dc);
		};

		for (int i = 0; i < 4; ++i) {
			line(i, (i + 1) % 4);
		}
		for (int i = 0; i < 4; ++i) {
			line(4 + i, 4 + (i + 1) % 4);
		}
		for (int i = 0; i < 4; ++i) {
			line(i, i + 4);
		}

		//for (int i = 0; i < 4; ++i) {
		//	line(i, 4 + (i + 2) % 4);
		//}
		for (int i = 0; i < 4; ++i) {
			line(i, 4 + (i + 1) % 4);
		}
		for (int i = 0; i < 4; ++i) {
			line(i, 4 + (i + 3) % 4);
		}

		line(0, 2);
		line(1, 3);
		line(4, 6);
		line(5, 7);
	}

	_mass.resize(_points.size(), 1.0f);
	_pointsVelocity.resize(_points.size());
	_pointsMoved.resize(_points.size());

	_mass[0] = std::numeric_limits<float>::max();
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	// update
	float dt = 1.0f / kFPS;
	
	for (int i = 0; i < _points.size(); ++i) {
		_pointsMoved[i] = _points[i] + _pointsVelocity[i] * dt;
	}

	int N = 20;
	float k = _stiffness;
	float k_tap = 1.0f - std::pow(1.0f - k, 1.0f / N);

	for (int j = 0; j < N; ++j) {
		// distance
		for (int i = 0; i < _dconstraints.size(); ++i) {
			float d = _dconstraints[i].length;
			int index0 = _dconstraints[i].index0;
			int index1 = _dconstraints[i].index1;
			glm::vec3 p0 = _pointsMoved[index0];
			glm::vec3 p1 = _pointsMoved[index1];


			float w0 = 1.0f / _mass[index0];
			float w1 = 1.0f / _mass[index1];
			float weight0 = w0 / (w0 + w1);
			float weight1 = w1 / (w0 + w1);

			glm::vec3 delta_p0 = -weight0 * (glm::distance(p0, p1) - d) * glm::normalize(p0 - p1);
			glm::vec3 delta_p1 = +weight1 * (glm::distance(p0, p1) - d) * glm::normalize(p0 - p1);

			_pointsMoved[index0] += delta_p0 * k_tap;
			_pointsMoved[index1] += delta_p1 * k_tap;
		}

		// floor

		{
			float k = 0.3;
			float k_tap = 1.0f - std::pow(1.0f - k, 1.0f / N);
			for (int i = 0; i < _points.size(); ++i) {
				glm::vec3 p = _pointsMoved[i];
				if (p.y - _floorConstraint.h < 0.0f) {
					float c = p.y - _floorConstraint.h;
					glm::vec3 grad_c = glm::vec3(0.0f, 1.0f, 0.0f);
					glm::vec3 delta_p = -c * grad_c;
					_pointsMoved[i] += delta_p * k_tap;
				}
			}
		}
	}


	// velocity update
	for (int i = 0; i < _points.size(); ++i) {
		_pointsVelocity[i] = (_pointsMoved[i] - _points[i]) / dt;

		if (std::numeric_limits<float>::max() <= _mass[i]) {
			continue;
		}

		// floor collision detection and friction
		// not work well
		//{
		//	float friction = 0.8f;
		//	float eps = 0.0001f;
		//	glm::vec3 p = _pointsMoved[i];
		//	if (p.y - _floorConstraint.h <= eps) {
		//		// glm::vec3 friction_f = -_pointsVelocity[i] * friction;
		//		glm::vec3 collision_N(0.0f, 1.0f, 0.0f);
		//		glm::vec3 velocityWithoutN = (_pointsVelocity[i] - collision_N * glm::dot(_pointsVelocity[i], collision_N));
		//		if (glm::length2(velocityWithoutN) < 0.0001f) {
		//			continue;
		//		}

		//		glm::vec3 friction_f = -glm::normalize(velocityWithoutN) * friction;
		//		glm::vec3 a = friction_f / _mass[i];
		//		glm::vec3 deltaVelocity = a * dt;

		//		if (glm::length2(velocityWithoutN) < glm::length2(deltaVelocity)) {
		//			auto pv = _pointsVelocity[i];
		//			auto v = collision_N * glm::dot(_pointsVelocity[i], collision_N);
		//			// printf("{%.3f, %.3f, %.3f} => {%.3f, %.3f, %.3f}\n", pv.x, pv.y, pv.z, v.x, v.y, v.z);
		//			_pointsVelocity[i] = v;
		//		}
		//		else {
		//			_pointsVelocity[i] += deltaVelocity;
		//		}
		//	}
		//}


		// ma = f
		// a = f / m 
		// a = (mg) / m
		// a = g
		_pointsVelocity[i] = _pointsVelocity[i] + glm::vec3(0.0f, -9.8f * dt, 0.0f);
	}

	for (int i = 0; i < _points.size(); ++i) {
		_points[i] = _pointsMoved[i];
	}

	// drawing
	ofEnableDepthTest();

	ofClear(0);

	_camera.begin();
	// floor
	ofPushMatrix();
	ofTranslate(0, _floorConstraint.h, 0);
	ofRotateZ(90.0f);
	ofSetColor(64);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();


	ofPushMatrix();
	ofDrawAxis(50);
	ofPopMatrix();

	ofSetColor(ofColor::orange);
	for (int i = 0; i < _points.size(); ++i) {
		ofDrawSphere(toOf(_points[i]), 0.02);
	}

	ofSetColor(255);
	for (int i = 0; i < _dconstraints.size() ; ++i) {
		int index0 = _dconstraints[i].index0;
		int index1 = _dconstraints[i].index1;
		ofDrawLine(toOf(_points[index0]), toOf(_points[index1]));
	}

	_camera.end();


	ofDisableDepthTest();
	ofSetColor(255);

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());

	if (ImGui::Button("initialize")) {
		initialize();
	}
	ImGui::SliderFloat("h", &_floorConstraint.h, -2.0f, 2.0);
	ImGui::SliderFloat("stiffness", &_stiffness, 0.0f, 1.0f);

	auto wp = ImGui::GetWindowPos();
	auto ws = ImGui::GetWindowSize();
	ofRectangle win(wp.x, wp.y, ws.x, ws.y);

	ImGui::End();
	ImGui::PopStyleColor();

	_imgui.end();

	if (win.inside(ofGetMouseX(), ofGetMouseY())) {
		_camera.disableMouseInput();
	}
	else {
		_camera.enableMouseInput();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
