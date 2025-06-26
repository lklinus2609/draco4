
set(SOURCES draco_test.c)
add_executable(draco_test ${SOURCES})
target_link_libraries(draco_test soem)
install(TARGETS draco_test DESTINATION bin)
