message("============world subdirectory start===============")
message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message("PROJECT_SOURCE_DIR      : ${PROJECT_SOURCE_DIR}")
message("GOOGLE_DIR          	 : ${GOOGLE_DIR}\n")

include_directories(./include)
set(DIR_SRCS ./src/world.c)

add_library(world SHARED ${DIR_SRCS})
