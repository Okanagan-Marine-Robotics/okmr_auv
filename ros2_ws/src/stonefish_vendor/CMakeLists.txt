cmake_minimum_required(VERSION 3.8)
project(stonefish_vendor)

find_package(ament_cmake REQUIRED)

set(STONEFISH_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/stonefish)

find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
find_package(OpenMP)

add_subdirectory(${STONEFISH_ROOT_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/stonefish_build)


install(DIRECTORY ${STONEFISH_ROOT_DIR}/Library/include/
        DESTINATION include/Stonefish
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/stonefish_build/version.h)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stonefish_build/version.h
            DESTINATION include/Stonefish)
endif()

install(DIRECTORY ${STONEFISH_ROOT_DIR}/3rdparty/
        DESTINATION include/Stonefish/3rdparty)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/stonefish_build/
        DESTINATION lib
        FILES_MATCHING PATTERN "libStonefish.so*")


file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/StonefishConfig.cmake
"
include(CMakeFindDependencyMacro)
find_dependency(OpenGL REQUIRED)
find_dependency(Freetype REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
find_package(OpenMP)

if(NOT TARGET Stonefish::Stonefish)
  add_library(Stonefish::Stonefish SHARED IMPORTED)
  get_filename_component(_STONEFISH_PREFIX \"\${CMAKE_CURRENT_LIST_DIR}/../../..\" ABSOLUTE)
  
  set_target_properties(Stonefish::Stonefish PROPERTIES
    IMPORTED_LOCATION \"\${_STONEFISH_PREFIX}/lib/libStonefish.so\"
    INTERFACE_INCLUDE_DIRECTORIES \"\${_STONEFISH_PREFIX}/include;\${_STONEFISH_PREFIX}/include/Stonefish;\${_STONEFISH_PREFIX}/include/Stonefish/3rdparty\"
  )

  set_property(TARGET Stonefish::Stonefish APPEND PROPERTY
    INTERFACE_COMPILE_DEFINITIONS BT_USE_DOUBLE_PRECISION)

  set_property(TARGET Stonefish::Stonefish APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES OpenGL::GL Freetype::Freetype \${SDL2_LIBRARIES})
    
  if(OpenMP_CXX_FOUND)
    set_property(TARGET Stonefish::Stonefish APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES OpenMP::OpenMP_CXX)
  endif()
  
  set_property(TARGET Stonefish::Stonefish APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES \${SDL2_INCLUDE_DIRS})
endif()
")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/StonefishConfig.cmake
        DESTINATION lib/cmake/Stonefish)

ament_package()