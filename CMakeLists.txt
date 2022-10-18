cmake_minimum_required(VERSION 3.4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(QTrackballCameraController)

find_package(Qt6 COMPONENTS Core Gui 3DRender 3DInput 3DLogic Qml REQUIRED)

add_library(${PROJECT_NAME} SHARED src/QTrackballCameraController.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

qt_add_qml_module(${PROJECT_NAME} URI QTrackballCameraController
    URI Qt3D.Extras.TrackBallCameraController
    VERSION 1.0
    SOURCES src/QTrackballCameraController.h src/QTrackballCameraController.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Core Qt6::Gui Qt6::3DRender Qt6::3DInput Qt6::3DLogic Qt6::Qml)

target_include_directories(${PROJECT_NAME} PUBLIC src)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "src/QTrackballCameraController;src/QTrackballCameraController.h")

install(TARGETS ${PROJECT_NAME} PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Qt3DExtras)
