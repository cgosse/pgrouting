add_library(bd_astar OBJECT bdastar.h bdastar.c MinHeap.h MinHeap.cpp BiDirAStar.h bdastar_core.cpp BiDirAStar.cpp)
# specify that the c++ wrapper needs openmp enabled to run in parallel

