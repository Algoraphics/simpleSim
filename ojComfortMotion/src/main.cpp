// Author: Jonathan Sprinkle
// Created based on a demonstration/tutorial from the openJaus
// project. Unclear (at best) how supported it is, or whether it
// will work...

// even though not much is in there, maybe I will develop a class
// for an openJaus component to make life easier?
#include "comfortMotion.h"
#include <iostream>
#include <cstdlib>

int main(void) {
	// first, declare the controller
	OjCmpt mpComponent;

	//  std::cout << "Press [1] to choose a test set." << std::endl;
	//  char c = getchar( );
	//  switch ( c )
	//  {
	//    case '1':
	//      // construct and run test case 1
	//      std::cout << "Received 1..." << std::endl;
	//      break;
	//    case 27: // escape key
	//      std::cout << "Escape pressed, aborting." << std::endl;
	//      break;
	//    default:
	//      std::cout << "Invalid choice!" << std::endl;
	//      break;
	//  }
	//
	//  if( c != 27 )
	//  {
	mpComponent = VehicleController::create("MotionProfile Component");
	VehicleController::run(mpComponent);
	VehicleController::destroy(mpComponent);
	//  }
	//  else {
	//    std::cout << "Exiting." << std::endl;
	//  }

	return EXIT_SUCCESS;

	//	else {
	//		std::cout << "Something bad happened..." << std::endl;
	//	}
	//
	//
	//	return EXIT_SUCCESS;

	//		ojCmptRun(myController);
	//
	//		// and quit when you hit enter...
	//		getchar( );
	//
	//		ojCmptDestroy(myController);



	return EXIT_SUCCESS;

}