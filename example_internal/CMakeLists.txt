add_executable(moteus_chatter main.cpp)

target_link_libraries(moteus_chatter ${LIBRARY_NAME})

install(
  TARGETS moteus_chatter
  # EXPORT ${PROJECT_EXPORT} # uncomment in order to export target
  RUNTIME DESTINATION bin
  COMPONENT "${INSTALL_BIN_DIR}"
)

