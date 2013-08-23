# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

INCLUDE_DIRECTORIES(../libtutorial/include ../libjaus/include ../libopenJaus/include ./include )
SET( OJTUTORIALSRCS 
	./src/main.cpp
	./src/tutorial.cpp
 )

SET(OJTUTORIALHEADERS ./include/tutorial.h
)

set(EXECUTABLE_OUTPUT_PATH .)


rosbuild_ADD_EXECUTABLE(ojTutorial ${OJTUTORIALSRCS} ${OJTUTORIALHEADERS})
TARGET_LINK_LIBRARIES(ojTutorial tutorial openJaus jaus pthread curses)
