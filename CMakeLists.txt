if(CONFIG_ZMK_INPUT_PROCESSOR_MOVEMENT_THRESHOLD)
  target_sources(app PRIVATE src/movement_threshold.c)
  target_include_directories(app PRIVATE include)
endif()
