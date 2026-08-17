project(SoftRobotsBindings)

set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_SoftRobotsBaseConstraint.h
)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_SoftRobotsBaseConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Module_SoftRobots.cpp
)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS Plugin Bindings.Sofa)
endif()

SP3_add_python_module(
    TARGET       ${PROJECT_NAME}
    PACKAGE      SoftRobots
    MODULE       SoftRobots
    DESTINATION  Sofa
    SOURCES      ${SOURCE_FILES}
    HEADERS      ${HEADER_FILES}
    DEPENDS      SofaPython3::Plugin SofaPython3::Bindings.Sofa SoftRobots
)

target_sources(SoftRobotsBindings
  PRIVATE
    Binding_SoftRobotsBaseConstraint.cpp
)
message("-- SofaPython3 bindings for SoftRobots will be created.")
