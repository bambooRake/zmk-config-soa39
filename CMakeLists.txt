if(CONFIG_ZMK_INPUT_PROCESSOR_POINTER_INERTIA)
    zephyr_library()
    zephyr_library_sources(src/input_processor_pointer_inertia.c)
endif()
