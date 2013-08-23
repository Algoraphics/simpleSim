# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

INCLUDE_DIRECTORIES(../libazcar/include ./include )
SET( OJCOMFORTMOTIONSRCS 
	./src/main.cpp
	./src/comfortMotion.cpp
 )

SET(OJCOMFORTMOTIONHEADERS 
	./include/comfortMotion.h
	./include/beginner_tutorials/steeringControl.h
)

set(EXECUTABLE_OUTPUT_PATH .)

rosbuild_ADD_EXECUTABLE(ojROSComfort ${OJCOMFORTMOTIONSRCS} ${OJCOMFORTMOTIONHEADERS})
TARGET_LINK_LIBRARIES(ojROSComfort m azcar openJaus jaus pthread curses)
