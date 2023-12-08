cmake_minimum_required(VERSION 3.5)
project(HelloWorld)

#设定C99标准
set(CMAKE_C_STANDARD 99)
message("==================打印信息使用======================")
#指定要包含的头文件路径，这里指出了路径，后面在代码中就可以直接用一层包含头文件就可以了
#不需要在类似中的包含 #include “…/include/xxx.h”；直接 #include "xxx.h"即可
include_directories(hello/include world/include thirdparty/include)#
set(GOOGLE_DIR ${PROJECT_SOURCE_DIR}/world)
#指定链接动态库的路径
link_directories(thirdparty/lib)

#设定DIR_SRCS变量值为 ./src/main.c
set(DIR_SRCS ./src/main.c)

#进入子文件夹hello，去执行子文件夹下的CMakeLists.txt
add_subdirectory(hello)
add_subdirectory(world)

#指定生成的项目名为HelloWorld，DIR_SRCS为其所依赖的源文件
add_executable(HelloWorld ${DIR_SRCS})
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/thirdparty/lib)
target_link_libraries(HelloWorld hello world calc_library)
