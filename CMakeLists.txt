##---------------------------------------------------------------------------------------------------------------------
##  MOTION_PLANNING
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(motion_planning VERSION 0.3 LANGUAGES C CXX)

##########################################
######		Configure project		######
##########################################

# Others
set(BUILD_TOOLS OFF)
set(BUILD_EXAMPLES ON)

#########################################
###### Loading 3rd party libraries.######
#########################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND motion_planning_DEFINITIONS "")
set(motion_planning_COMPILE_OPTIONS "-Wall")


#########################################
## Load opencv
find_package(OpenCV 3 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

#########################################
## Load PCL
find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	# -march=native -msse4.2 -mfpmath=sse -DDISABLE_PCAP-DDISABLE_PNG-DDISABLE_LIBUSB_1_0
	list(APPEND motion_planning_DEFINITIONS "ENABLE_PCL")
	list(APPEND motion_planning_DEFINITIONS "DISABLE_PCAP-DDISABLE_PNG")
	list(APPEND motion_planning_DEFINITIONS "DISABLE_LIBUSB_1_0")
	set(motion_planning_COMPILE_OPTIONS "${motion_planning_COMPILE_OPTIONS} -march=native -msse4.2 -mfpmath=sse -L/usr/lib/x86_64-linux-gnu")
	if(${PCL_VERSION} VERSION_GREATER 1.8)
		list(APPEND motion_planning_DEFINITIONS "HAS_PCL_1_8")
	else()
		list(APPEND motion_planning_DEFINITIONS "HAS_PCL_1_7")
	endif()
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
file(GLOB_RECURSE RGDBTOOLS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE RGDBTOOLS_HEADER_FILES "include/*.h" "include/*.inl" "include/*.hpp")
 
add_library(${PROJECT_NAME} SHARED ${RGDBTOOLS_HEADER_FILES} ${RGDBTOOLS_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${motion_planning_DEFINITIONS})
string(REPLACE " " ";" motion_planning_COMPILE_OPTIONS_REPLACED_FLAGS ${motion_planning_COMPILE_OPTIONS})
target_compile_options(${PROJECT_NAME} PUBLIC ${motion_planning_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_motion_planning
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

#########################################
######		   	Tools 		   	   ######
#########################################
if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			Examples 		   ######
#########################################
if(${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS motion_planning
        EXPORT motion_planningTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    motion_planningConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )


# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.hpp")

export(TARGETS motion_planning NAMESPACE motion_planning:: FILE motion_planningTargets.cmake)

# This generates motion_planningTargets.cmake
install(EXPORT motion_planningTargets
    FILE motion_planningTargets.cmake
    NAMESPACE motion_planning::
    DESTINATION lib/cmake/motion_planning
    )

configure_file(motion_planningConfig.cmake.in motion_planningConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/motion_planningConfig.cmake"
        DESTINATION lib/cmake/motion_planning
        )



# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
 
