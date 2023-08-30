# CMakeList.txt : CMake project for JacobianCore, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

find_package(Torch REQUIRED)

# Set optimization level
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

add_compile_definitions(EXPORT_LIB)
# Add source to this project's library
add_library(JacobianCore SHARED JacobianCore.cpp
                                JacobianCore.h
                                J_Interface.cpp
                                J_Interface.h
                                DebugCPP.cpp
                                DebugCPP.h)

add_executable(JacobianJig   JacobianJig.cpp)
add_executable(JacobianEx    JacobianEx.cpp
                             JacobianCore.cpp 
                             JacobianCore.h)

add_executable(JitModules     JitModules.cpp)

add_executable(TandomTest   TandomTest.cpp)

target_link_libraries (JacobianCore "${TORCH_LIBRARIES}")
target_link_libraries (JacobianEx "${TORCH_LIBRARIES}")
target_link_libraries (JitModules "${TORCH_LIBRARIES}")

# Needed to link the torch libraries
if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET JacobianCore
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:JacobianCore>)

  add_custom_command(TARGET JacobianEx
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:JacobianEx>)

  add_custom_command(TARGET JitModules
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:JitModules>)


endif (MSVC)