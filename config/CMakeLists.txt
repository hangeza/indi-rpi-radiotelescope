cmake_minimum_required(VERSION 2.6)

PROJECT(systemd_units)

include(GNUInstallDirs)

# tell cmake where to install the files
install(FILES 
indiserver.service
ratsche.service
DESTINATION /usr/lib/systemd/system)

install(CODE "execute_process(COMMAND systemctl daemon-reload)")
