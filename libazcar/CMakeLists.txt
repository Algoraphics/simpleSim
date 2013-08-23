# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

SET( LIBAZCARSRCS ./src/motionCommand.c
	./src/predictionMessage.c
	./src/setMotionProfileMessage.c
	./src/state.c
	./src/reportCurvatureMessage.c
	./src/queryVelodyneDataMessage.c
	./src/velodyneDataMessage.c
	./src/velodyneDataSample.c
	./src/velodyneDataPoint.c
	./src/verifyPathMessage.c
	./src/wayPoint.c
	./src/wayPointRequestMessage.c
)

SET(LIBAZCARHEADERS 
	./include/common.h
	./include/motionCommand.h
	./include/predictionMessage.h
	./include/setMotionProfileMessage.h 
	./include/state.h
	./include/reportCurvatureMessage.h 
	./include/queryVelodyneDataMessage.h
	./include/velodyneDataMessage.h
	./include/velodyneDataSample.h
	./include/velodyneDataPoint.h
	./include/verifyPathMessage.h
	./include/wayPoint.h
	./include/wayPointRequestMessage.h
)

INCLUDE_DIRECTORIES(./include)

add_library(azcar STATIC ${LIBAZCARSRCS} ${LIBAZCARHEADERS}) 
