add_library(bd_astar_bulk OBJECT bdastar_bulk.h bdastar_bulk.c MinHeapBulk.h MinHeapBulk.cpp BiDirAStar_bulk.h bdastar_core_bulk.cpp BiDirAStar_bulk.cpp)

# add openMP flags for compilation. link flags are set in the root CMakeLists.txt
if(APPLE)
    set_target_properties(bd_astar_bulk PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS} -openmp"  )
else(APPLE)
    set_target_properties(bd_astar_bulk PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS} -fopenmp"  )
endif(APPLE)


