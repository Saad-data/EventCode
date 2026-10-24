# Copyright (c) Prophesee S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(silky_barcode_demo VERSION 1.0 LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(MetavisionSDK COMPONENTS core stream REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui imgcodecs videoio)
find_package(Threads REQUIRED)

add_executable(silky_barcode_demo
    src/main.cpp
    src/code128.cpp
)
target_include_directories(silky_barcode_demo PRIVATE src)
target_link_libraries(silky_barcode_demo PRIVATE
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
    Threads::Threads
)
target_compile_options(silky_barcode_demo PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME code128_self_test COMMAND silky_barcode_demo --self-test)
