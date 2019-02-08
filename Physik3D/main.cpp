#include "Physik3DApp.h"

int main() {
	
	// allocation
	auto app = new Physik3DApp();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}