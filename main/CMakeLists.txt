# SPDX-FileCopyrightText: 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

# Select source file based on menuconfig choice
if(CONFIG_EXAMPLE_SMOKE_TEST)
    set(MAIN_SRCS "smoke_test.cpp")
elseif(CONFIG_EXAMPLE_SIMPLE_COLORS)
    set(MAIN_SRCS "../examples/01_basic/simple_colors/main/main.cpp")
elseif(CONFIG_EXAMPLE_GRADIENTS)
    set(MAIN_SRCS "../examples/01_basic/gradients/main/main.cpp")
elseif(CONFIG_EXAMPLE_BRIGHTNESS)
    set(MAIN_SRCS "../examples/01_basic/brightness/main/main.cpp")
elseif(CONFIG_EXAMPLE_TWO_HORIZONTAL)
    set(MAIN_SRCS "../examples/02_multi_panel/two_horizontal/main/main.cpp")
elseif(CONFIG_EXAMPLE_THREE_HORIZONTAL)
    set(MAIN_SRCS "../examples/02_multi_panel/three_horizontal/main/main.cpp")
elseif(CONFIG_EXAMPLE_SERPENTINE)
    set(MAIN_SRCS "../examples/02_multi_panel/serpentine/main/main.cpp")
elseif(CONFIG_EXAMPLE_LVGL_DEMO)
    set(MAIN_SRCS "../examples/03_lvgl/simple_demo/main/main.cpp"
                  "../examples/03_lvgl/simple_demo/main/lv_port_hub75.cpp")
else()
    # Fallback to smoke test if no config set (should not happen)
    set(MAIN_SRCS "smoke_test.cpp")
endif()

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "." "../examples/common"
    REQUIRES hub75
)

# Add LVGL dependency if LVGL example selected
if(CONFIG_EXAMPLE_LVGL_DEMO)
    # LVGL will be pulled in via idf_component.yml in the example directory
    # when building standalone. For root builds, LVGL must be available.
    # Check if component exists before trying to use it
    idf_build_get_property(build_components BUILD_COMPONENTS)
    if("lvgl" IN_LIST build_components)
        idf_component_get_property(lvgl_lib lvgl COMPONENT_LIB)
        target_link_libraries(${COMPONENT_LIB} PUBLIC ${lvgl_lib})
    else()
        message(FATAL_ERROR "LVGL component not found. To build the LVGL example:\n"
                            "  Option 1: Build standalone: cd examples/03_lvgl/simple_demo && idf.py build\n"
                            "  Option 2: Add LVGL to root idf_component.yml\n"
                            "  Option 3: Select a different example via menuconfig")
    endif()
endif()
