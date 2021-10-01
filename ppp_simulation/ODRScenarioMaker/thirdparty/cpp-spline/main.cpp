#include <iostream>
#include "./CatmullRom.h"
#include "./BSpline.h"

using namespace std;

void demo_catmullrom();

int main() {
	demo_catmullrom();

	return 0;
}


void demo_catmullrom() {
	Curve* curve = new BSpline();
	curve->set_steps(100); // generate 100 interpolate points between the last 4 way points
	curve->add_way_point(Vector(1, 1, 0));
	curve->add_way_point(Vector(2, 3, 0));
	curve->add_way_point(Vector(3, 2, 0));
	curve->add_way_point(Vector(4, 6, 0));
	std::cout << "nodes: " << curve->node_count() << std::endl;
	std::cout << "total length: " << curve->total_length() << std::endl;
	for (int i = 0; i < curve->node_count(); ++i) {
		std::cout << curve->node(i).x << "," << curve->node(i).y << std::endl;
	}
	delete curve;
}
