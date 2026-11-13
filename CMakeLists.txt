cmake_minimum_required(VERSION 3.16)

# Components live in ./components (auto-discovered). Host-only code (tests/, tools/, ml/) is not part of the build.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(aai16_edge_node)
