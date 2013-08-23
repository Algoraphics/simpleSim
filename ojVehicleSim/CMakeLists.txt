# Author: Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.

INCLUDE_DIRECTORIES(../libjaus/include ../libopenJaus/include ./include )
SET( OJVEHICLESIMSRCS ./src/getLine.c
./src/gpos.c
./src/main.c
./src/pd.c
./src/properties.c
./src/utm/cproj.c
./src/utm/greatCircle.c
./src/utm/pointLla.c
./src/utm/pointUtm.c
./src/utm/utmfor.c
./src/utm/utminv.c
./src/utm/utmLib.c
./src/vehicle.c
./src/vehicleSim.c
./src/vss.c
./src/wd.c
 )

SET(OJVEHICLESIMHEADERS ./include/getline.h
./include/gpos.h
./include/gui.h
./include/pd.h
./include/properties.h
./include/utm/cproj.h
./include/utm/greatCircle.h
./include/utm/pointLla.h
./include/utm/pointUtm.h
./include/utm/proj.h
./include/utm/utmLib.h
./include/vehicle.h
./include/vehicleSim.h
./include/vss.h
./include/wd.h
)

set(EXECUTABLE_OUTPUT_PATH .)


ADD_EXECUTABLE(ojVehicleSim ${OJVEHICLESIMSRCS} ${OJVEHICLESIMHEADERS})
TARGET_LINK_LIBRARIES(ojVehicleSim openJaus jaus pthread curses)
