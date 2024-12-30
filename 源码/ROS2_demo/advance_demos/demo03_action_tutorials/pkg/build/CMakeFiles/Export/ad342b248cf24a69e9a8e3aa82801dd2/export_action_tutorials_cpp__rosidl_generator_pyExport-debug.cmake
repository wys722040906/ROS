#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "action_tutorials_cpp::action_tutorials_cpp__rosidl_generator_py" for configuration "Debug"
set_property(TARGET action_tutorials_cpp::action_tutorials_cpp__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(action_tutorials_cpp::action_tutorials_cpp__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libaction_tutorials_cpp__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "libaction_tutorials_cpp__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets action_tutorials_cpp::action_tutorials_cpp__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_action_tutorials_cpp::action_tutorials_cpp__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libaction_tutorials_cpp__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
