set(SOURCES
    main.c
    system.c
)

add_executable(NESEmulator ${SOURCES})
target_link_libraries(NESEmulator ${SDL2_LIBRARIES})
