#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "teleop_controllers::controller_manager" for configuration "Release"
set_property(TARGET teleop_controllers::controller_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(teleop_controllers::controller_manager PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcontroller_manager.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS teleop_controllers::controller_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_teleop_controllers::controller_manager "${_IMPORT_PREFIX}/lib/libcontroller_manager.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
