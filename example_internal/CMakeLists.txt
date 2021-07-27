add_executable(pi3hat_tool pi3hat_tool.cc)

target_link_libraries(pi3hat_tool ${LIBRARY_NAME})

install(
  TARGETS pi3hat_tool
  # EXPORT ${PROJECT_EXPORT} # uncomment in order to export target
  RUNTIME DESTINATION bin
  COMPONENT "${INSTALL_BIN_DIR}"
)

