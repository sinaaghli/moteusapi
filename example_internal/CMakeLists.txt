add_executable(moteus_chatter main.cpp)
target_link_libraries(moteus_chatter ${LIBRARY_NAME})

install(
  TARGETS moteus_chatter
  # EXPORT ${PROJECT_EXPORT} # uncomment in order to export target
  RUNTIME DESTINATION bin
  COMPONENT "${INSTALL_BIN_DIR}"
)

add_executable(main2 main2.cpp)
target_link_libraries(main2 ${LIBRARY_NAME})

add_executable(main3 main3.cpp)
target_link_libraries(main3 ${LIBRARY_NAME})