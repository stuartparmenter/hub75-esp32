# SPDX-FileCopyrightText: 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

idf_component_register(
    SRCS "main.cpp" "lvgl_demo_ui.c"
    INCLUDE_DIRS "." "../../../common"
    REQUIRES hub75 lvgl
)
