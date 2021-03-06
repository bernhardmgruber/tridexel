cmake_minimum_required(VERSION 3.0)

project(tridexel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find packages
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

find_package(glm CONFIG REQUIRED)
find_package(GTest MODULE REQUIRED)

# library
file(GLOB source_files src/*)
add_library(${PROJECT_NAME} ${source_files})
source_group("" FILES ${source_files})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${source_files})

target_link_libraries(${PROJECT_NAME} PRIVATE
	${Boost_LIBRARIES}
	glm
)

if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

# tests
file(GLOB test_files test/*)
add_executable(tests ${test_files})
source_group("" FILES ${test_files})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${test_files})

target_link_libraries(tests PRIVATE
	${PROJECT_NAME}
	GTest::GTest GTest::Main
	${Boost_LIBRARIES}
	glm
)

# clang-format
find_program(CLANG_FORMAT NAMES "clang-format")
if(NOT CLANG_FORMAT)
	message("clang-format not found")
else()
	message("clang-format found: ${CLANG_FORMAT}")
	add_custom_target(clang-format COMMAND ${CLANG_FORMAT} -style=file -i ${source_files})
endif()

# clang-tidy
find_program(CLANG_TIDY NAMES "clang-tidy")
if(NOT CLANG_TIDY)
	message("clang-tidy not found")
else()
	message("clang-tidy found: ${CLANG_TIDY}")
	set_target_properties(${PROJECT_NAME} PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY}")
endif()
