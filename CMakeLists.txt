cmake_minimum_required(VERSION 3.4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(QTrackballCameraController)

find_package(Qt6 COMPONENTS Core Gui 3DRender 3DInput 3DLogic REQUIRED)

qt_wrap_cpp(MOC src/QTrackballCameraController.h)

add_library(${PROJECT_NAME} SHARED src/QTrackballCameraController.cpp ${MOC})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Core Qt6::Gui Qt6::3DRender Qt6::3DInput Qt6::3DLogic)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "src/QTrackballCameraController;src/QTrackballCameraController.h")

install(TARGETS ${PROJECT_NAME} PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Qt3DExtras)
