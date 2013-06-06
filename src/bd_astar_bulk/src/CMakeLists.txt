add_library(bd_astar_bulk OBJECT bdastar_bulk.h bdastar_bulk.c MinHeapBulk.h MinHeapBulk.cpp BiDirAStar_bulk.h bdastar_core_bulk.cpp BiDirAStar_bulk.cpp)

get_target_property(TEMP bd_astar_bulk COMPILE_FLAGS)
if(TEMP STREQUAL "TEMP-NOTFOUND")
  SET(TEMP "") # set to empty string
else()
  SET(TEMP "${TEMP} ") # a space to cleanly separate from existing content
endif()
# append the openmp flag for this library
if(APPLE)
    #clang uses this flag
    SET(TEMP "${TEMP} -openmp" )
else(APPLE)
    SET(TEMP "${TEMP} -fopenmp" )
endif(APPLE)

set_target_properties(bd_astar_bulk PROPERTIES COMPILE_FLAGS ${TEMP} )

