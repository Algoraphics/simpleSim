# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

INCLUDE_DIRECTORIES(../libtutorial/include ./include )
SET( OJTUTORIALSRCS 
	./src/main.cpp
	./src/tutorial.cpp
 )

SET(OJTUTORIALHEADERS ./include/tutorial.h
)

set(EXECUTABLE_OUTPUT_PATH .)


rosbuild_ADD_EXECUTABLE(ojGazeboBridge ${OJTUTORIALSRCS} ${OJTUTORIALHEADERS})
TARGET_LINK_LIBRARIES(ojGazeboBridge tutorial openJaus jaus pthread curses)
