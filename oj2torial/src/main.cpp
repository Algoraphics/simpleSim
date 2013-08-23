// Author: Sean Whitsitt
// Tutorial component, just sends a number back and forth
#include "tutorial.h"
#include <iostream>
#include <cstdlib>
#include <sstream>

int main(int argc, char **argv) {
	// first, declare the controller
	OjCmpt tutorialComponent;
	tutorialComponent = TutorialComponent::create("Tutorial Component");
	TutorialComponent::run(tutorialComponent);
	TutorialComponent::destroy(tutorialComponent);
	return EXIT_SUCCESS;
}
