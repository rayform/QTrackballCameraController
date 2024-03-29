cmake_minimum_required(VERSION 3.16)
project(QTrackballCameraController)

find_package(Qt6 COMPONENTS Core Gui 3DRender 3DInput 3DLogic Qml REQUIRED)

qt_standard_project_setup()

qt_add_library(${PROJECT_NAME} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QTrackballCameraController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QTrackballCameraController.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/QTrackballCameraController
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt6::3DRender
    Qt6::3DInput
    Qt6::3DLogic
)

qt_add_qml_module(${PROJECT_NAME}
    URI Qt3D.Extras.TrackballCameraController
    CLASS_NAME TrackballCameraController
    VERSION 1.0
    SOURCES src/QTrackballCameraController.h src/QTrackballCameraController.cpp
    OUTPUT_DIRECTORY Qt3D/Extras/TrackballCameraController
    DEPENDENCIES Qt3DRender Qt3DInput Qt3DLogic
)

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER
    src/QTrackballCameraController.h src/QTrackballCameraController)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Qt3DExtras
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# export plugin
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Qt3D" DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(EXPORT ${PROJECT_NAME}Config DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
