function(nav2_generate_tree_nodes_xml)
  # Validate arguments
  cmake_parse_arguments(
    ARG
    "SKIP_INSTALL"
    "GENERATED_DIR;TREENODES_FILE;PLUGIN_LIST_TEMPLATE_FILE;INSTALL_PATH"
    "PLUGIN_LIBS"
    ${ARGN}
  )
  if(NOT ARG_GENERATED_DIR)
    set(ARG_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen")
  endif()
  if(NOT ARG_TREENODES_FILE)
    set(ARG_TREENODES_FILE "${PROJECT_NAME}_tree_nodes.xml")
  endif()
  if(NOT ARG_PLUGIN_LIST_TEMPLATE_FILE)
    if(NOT nav2_behavior_tree_DIR)
      set(ARG_PLUGIN_LIST_TEMPLATE_FILE "${CMAKE_SOURCE_DIR}/cmake/plugin_list.txt.in")
    else()
      set(ARG_PLUGIN_LIST_TEMPLATE_FILE "${nav2_behavior_tree_DIR}/plugin_list.txt.in")
    endif()
  endif()
  if(NOT ARG_INSTALL_PATH)
    set(ARG_INSTALL_PATH "share/${PROJECT_NAME}")
  endif()

  # Make sure the templates to use are available
  if(NOT EXISTS "${ARG_PLUGIN_LIST_TEMPLATE_FILE}")
    message(FATAL_ERROR "Can't find ${ARG_PLUGIN_LIST_TEMPLATE_FILE}. Maybe reinstall nav2_behavior_tree package.")
  endif()

  if(NOT ARG_PLUGIN_LIBS)
    message(FATAL_ERROR "PLUGIN_LIBS option is required.")
  endif()
  list(SORT ARG_PLUGIN_LIBS)

  # retrieve version information from <package>.xml file
  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()
  string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
  set(VERSION_STR ${${PROJECT_NAME}_VERSION})

  # parse version information from the version string
  if(NOT VERSION_STR MATCHES "([0-9]+)\.([0-9]+)\.([0-9]+)")
    message(FATAL_ERROR "Version string must be of format MAJOR.MINOR.PATCH")
  endif()
  set(VERSION_MAJOR ${CMAKE_MATCH_1})
  set(VERSION_MINOR ${CMAKE_MATCH_2})
  set(VERSION_PATCH ${CMAKE_MATCH_3})

  set(GENERATED_TREENODES_FILE "${ARG_GENERATED_DIR}/${ARG_TREENODES_FILE}")

  string(REPLACE ";" "\n" plugin_libs_one_per_line "${ARG_PLUGIN_LIBS}")
  configure_file(${ARG_PLUGIN_LIST_TEMPLATE_FILE} ${GENERATED_DIR}/plugins_list.txt @ONLY)

  add_custom_command(
    OUTPUT "${GENERATED_TREENODES_FILE}"
    COMMAND  ${CMAKE_COMMAND} -E env LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}
      $<TARGET_FILE:nav2_behavior_tree::generate_tree_nodes_xml_cli>
      "${ARG_GENERATED_DIR}/plugins_list.txt"
      "${GENERATED_TREENODES_FILE}"
    POST_BUILD
    DEPENDS
      nav2_behavior_tree::generate_tree_nodes_xml_cli
      "${ARG_PLUGIN_LIBS}"
      "${GENERATED_DIR}/plugins_list.txt"
    COMMENT "Generating groot tree nodes description file ${GENERATED_TREENODES_FILE}, using ${ARG_GENERATED_DIR}/plugins_list.txt"
  )

  add_custom_target("nav2_generate_treenodes_file__${PROJECT_NAME}" ALL
    DEPENDS
      nav2_behavior_tree::generate_tree_nodes_xml_cli
      "${GENERATED_DIR}/plugins_list.txt"
      "${GENERATED_TREENODES_FILE}"
  )

  if(NOT ARG_SKIP_INSTALL)
    install(FILES
      "${GENERATED_TREENODES_FILE}"
      "${GENERATED_DIR}/plugins_list.txt"
      DESTINATION "${ARG_INSTALL_PATH}")
  endif()
endfunction()

