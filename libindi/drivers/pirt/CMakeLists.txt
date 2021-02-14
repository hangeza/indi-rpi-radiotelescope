########## VSTW Radebeul Radiotelescope driver ##############
#add_executable(radiovstw simplescope.cpp)

#target_link_libraries(radiovstw indidriver)


########### INDI Pi Radio Telescope driver ##############
SET(pirt_SRC
    pirt.cpp
    gpioif.cpp
    encoder.cpp)

add_executable(indi_pirt ${pirt_SRC})
target_link_libraries(indi_pirt indidriver "-lpigpiod_if2" "-lrt")
#install(TARGETS indi_pirt RUNTIME DESTINATION bin)

