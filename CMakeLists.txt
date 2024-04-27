# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
#set(COMPONENT_EMBED_TXTFILES /home/teppo/esp_projects/thermostat/server_certs/ca_cert.pem)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(thermostat)
