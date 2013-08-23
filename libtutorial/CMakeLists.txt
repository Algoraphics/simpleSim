# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

SET( LIBTUTORIALSRCS ./src/oneNumberMessage.c
)

SET(LIBTUTORIALHEADERS 
	./include/common.h
	./include/oneNumberMessage.h
)

INCLUDE_DIRECTORIES(./include ../libopenJaus/include ../libjaus/include)

add_library(tutorial STATIC ${LIBTUTORIALSRCS} ${LIBTUTORIALHEADERS} ) 
