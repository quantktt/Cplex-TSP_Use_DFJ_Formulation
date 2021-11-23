cmake_minimum_required(VERSION 3.0)
project(TSPUsingDFJ)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD -m64 -Wall -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD")

#FIND_PACKAGE(COMPONENTS  REQUIRED)

set(ILOG_DIR /opt/ibm/ILOG/CPLEX_Studio1210)
set(CPLEX_DIR ${ILOG_DIR}/cplex)
set(SOLVER_DIR ${ILOG_DIR}/cpoptimizer)
set(CONCERT_DIR ${ILOG_DIR}/concert)
set(CPLEX_INCLUDE_DIR ${CPLEX_DIR}/include)
set(CONCERT_INCLUDE_DIR ${CONCERT_DIR}/include)

set(CPLEX_LIB_DIR ${CPLEX_DIR}/lib/x86-64_linux/static_pic)
set(CONCERT_LIB_DIR ${CONCERT_DIR}/lib/x86-64_linux/static_pic)


FIND_PATH(CPLEX_INCLUDE_DIR ilcplex/cplex.h
    HINTS ${CPLEX_INCLUDE_DIR})

FIND_PATH(CPLEX_CONCERT_INCLUDE_DIR ilconcert/iloenv.h
    HINTS ${CONCERT_INCLUDE_DIR})

message(STATUS "CPLEX Dir: ${CPLEX_DIR}")


FIND_PATH(CPLEX_BIN_DIR
            cplex
	    HINTS ${CPLEX_DIR}/bin/x86-64_linux #unix
	    ENV LIBRARY_PATH
	    ENV LD_LIBRARY_PATH
	    )
message(STATUS "CPLEX Bin Dir: ${CPLEX_BIN_DIR}")

SET(CPLEX_INCLUDE_DIRS ${CPLEX_INCLUDE_DIR} ${CPLEX_CONCERT_INCLUDE_DIR})
#SET(CPLEX_LIBRARIES ${CPLEX_CONCERT_LIBRARY} ${CPLEX_LIBRARY} ) #${CPLEX_ILOCPLEX_LIBRARY}
SET(CPLEX_LIBRARIES ${CPLEX_LIB_DIR} ${CONCERT_LIB_DIR} ) #${CPLEX_ILOCPLEX_LIBRARY}


SET(CPLEX_LIBRARIES "${CPLEX_LIBRARIES}") #;m;pthread

message(STATUS "lib: ${CPLEX_LIBRARIES}")


include_directories(${CMAKE_HOME_DIRECTORY} ${CPLEX_INCLUDE_DIRS})

link_directories(${CPLEX_LIBRARIES})

set( CommonLibs
        ilocplex
	concert
	cplex
	m
	pthread
	dl
)


file(GLOB source *.cpp)
add_executable(${PROJECT_NAME} ${source})

target_link_libraries( ${PROJECT_NAME} LINK_PUBLIC )
target_link_libraries( ${PROJECT_NAME} PRIVATE ${CommonLibs} )
