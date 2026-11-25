cmake_minimum_required(VERSION 3.16)

project(Practical_04_MavlinkGroundStation VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Network Charts Multimedia)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network Charts Multimedia)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    telemetrymodel.h
    telemetrymodel.cpp
    mavlinkmessageparser.h
    mavlinkmessageparser.cpp
    mavlinkmanager.h
    mavlinkmanager.cpp
    uavsimulator.h
    uavsimulator.cpp
    station.qrc
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Practical_04_MavlinkGroundStation
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(Practical_04_MavlinkGroundStation
        ${PROJECT_SOURCES}
    )
endif()

target_link_libraries(Practical_04_MavlinkGroundStation PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Charts
    Qt${QT_VERSION_MAJOR}::Multimedia
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Practical_04_MavlinkGroundStation)
endif()

add_executable(Practical_04_UavSimulator
    simulatormain.cpp
    uavsimulator.cpp
    uavsimulator.h
    mavlinkmessageparser.cpp
    mavlinkmessageparser.h
    station.qrc
)
target_link_libraries(Practical_04_UavSimulator PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
)

set_target_properties(Practical_04_MavlinkGroundStation Practical_04_UavSimulator
                      PROPERTIES WIN32_EXECUTABLE TRUE)
