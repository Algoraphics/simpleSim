# Author: Sean Whitsitt

INCLUDE_DIRECTORIES( ../libjaus/include ../libopenJaus/include ./include ../ojVehicleTester/include ../libazcar/include )

IF (IPHONESIM OR IPHONEDEVICE)
	MESSAGE( "NOTE: Not building the ojComfortMotion (we are cross compiling)")
ELSE( IPHONESIM OR IPHONEDEVICE )
	SET( OJCOMFORTMOTIONSRCS 
	./src/main.cpp
	./src/comfortMotion.cpp
	 )
	SET_SOURCE_FILES_PROPERTIES( ${OJCOMFORTMOTIONSRCS} COMPILE_FLAGS "-Wall -fprofile-arcs -ftest-coverage -fno-strict-aliasing -O2 -c -g")
SET( CMAKE_EXE_LINKER_FLAGS -lgcov)

	SET( OJCOMFORTMOTIONHEADERS 
	./include/comfortMotion.h
	)
	
	ADD_EXECUTABLE( ojComfortMotion ${OJCOMFORTMOTIONSRCS} ${OJCOMFORTMOTIONHEADERS} )
	TARGET_LINK_LIBRARIES( ojComfortMotion m azcar jaus openJaus pthread curses )
ENDIF (IPHONESIM OR IPHONEDEVICE)
