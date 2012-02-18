cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(qt_srcs    src/ParseTreeLablerForm.cpp src/PTNodeTableModel.cpp src/ColorMapTableModel.cpp)
set(qt_hdrs    src/ParseTreeLablerForm.h src/PTNodeTableModel.h src/ColorMapTableModel.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})


QT4_WRAP_UI(uis_h src/ParseTreeLablerForm.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(parse src/cfg3D.cpp)
rosbuild_add_executable(ParseTreeLablerF  ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(ParseTreeLablerF ${QT_LIBRARIES})

rosbuild_add_executable(makeGraph src/makeGraph.cpp)
rosbuild_add_executable(visualizeNighbors src/labelVisualizer.cpp)
rosbuild_add_executable(featuresGenerator src/featuresGenerator.cpp)
rosbuild_add_executable(singleBag2PCD src/singleBag2PCD.cpp)
rosbuild_add_executable(transferLabels src/transferLabels.cpp)
rosbuild_add_executable(featGenGen src/featGenGen.cpp)
#rosbuild_add_executable(featGen src/featGen.cpp)
rosbuild_add_executable(getLabelmapMajorityVoting src/getLabelmapMajorityVoting.cpp)
#rosbuild_add_executable(merger src/merger.cpp)
rosbuild_add_executable(labeler src/labeler.cpp)
#rosbuild_add_executable(rulesGenerator src/rulesGenerator.cpp)
#rosbuild_add_executable(generatedLearner src/generatedLearner.cpp)
#rosbuild_add_executable(LaptopLearner src/Laptop_generatedLearner.cpp)
#rosbuild_add_executable(FridgeLearner src/Fridge_generatedLearner.cpp)
#rosbuild_add_executable(generatedLearner ../bin/generatedLearner.cpp)
#rosbuild_add_executable(extract_planes src/extract_planes.cpp)
rosbuild_add_executable(CPU_generatedLearner src/CPU_generatedLearner.cpp)
#rosbuild_add_executable(Wall_Learner src/Wall_Learner.cpp)
rosbuild_add_executable(Monitor_generatedLearner src/Monitor_generatedLearner.cpp)
rosbuild_add_executable(Printer_generatedLearner src/Printer_generatedLearner.cpp)
#target_link_libraries(example ${PROJECT_NAME})
