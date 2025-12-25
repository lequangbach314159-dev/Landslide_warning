# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "config\\sdkconfig.cmake"
  "config\\sdkconfig.h"
  "landslide_warning.map"
  "project_elf_src_esp32.c"
  )
endif()
