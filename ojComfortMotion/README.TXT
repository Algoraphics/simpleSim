Author: Sean Whitsitt

The ojComfortMotion JAUS component is the implementation of the Passenger Comfort Controller and Hoffmann Steering Controller in OpenJAUS.  This component works for both the physical vehicle and the OpenJAUS simulator that is available in the ojVehicleSim directory.

This utility requires there to be a lod_x.mat and lod_y.mat file in the directory that it is run from.  Using the standard build procedure shown in the README-UA file in the directory above this directory, you can safely run the ojComfortMotion utility from this directory as follows:

../build/ojComfortMotion/ojComfortMotion

This utility also requires there to be an outputs directory in the directory that this utility is run from.  The utility will output debugging information and data into this directory.  The utility will crash when the vehicle reaches the end of the defined path if this directory is not present.

SOW: I have been meaning to add some options to this utility so that it can run with or without the outputs directory (or just create it if it does not exist), but I have not yet had time.