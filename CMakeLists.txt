if(CONFIG_ZMK_INPUT_PROCESSOR_POINTER_INERTIA OR CONFIG_ZMK_INPUT_PROCESSOR_AXIS_LOCK)
    zephyr_library()
    zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_POINTER_INERTIA
                                 src/input_processor_pointer_inertia.c)
    zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_AXIS_LOCK
                                 src/input_processor_axis_lock.c)
endif()
