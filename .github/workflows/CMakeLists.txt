cmake_minimum_required(VERSION 3.20)
project(LuxMonitor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

find_package(glfw3 CONFIG REQUIRED)
find_package(imgui  CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(LuxMonitor WIN32 main.cpp)

target_link_libraries(LuxMonitor PRIVATE
    glfw
    imgui::imgui
    implot::implot
    OpenGL::GL
    ws2_32
)

# copy runtime DLLs (nếu dùng vcpkg dynamic)
if(WIN32)
    add_custom_command(TARGET LuxMonitor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:LuxMonitor> $<TARGET_FILE_DIR:LuxMonitor>
        COMMAND_EXPAND_LISTS)
endif()
