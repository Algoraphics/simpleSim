# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

INCLUDE_DIRECTORIES(../libtutorial/include ../libjaus/include ../libopenJaus/include ./include )
SET( OJ2TORIALSRCS 
	./src/main.cpp
	./src/tutorial.cpp
 )

SET(OJ2TORIALHEADERS ./include/tutorial.h
)

set(EXECUTABLE_OUTPUT_PATH .)


ADD_EXECUTABLE(oj2torial ${OJ2TORIALSRCS} ${OJ2TORIALHEADERS})
TARGET_LINK_LIBRARIES(oj2torial tutorial openJaus jaus pthread curses)
