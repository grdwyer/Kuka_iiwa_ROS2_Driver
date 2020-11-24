cmake_minimum_required(VERSION 3.5)
project(fri_client_sdk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(
 include
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/base/friClientApplication.cpp
	src/client_lbr/friLBRClient.cpp
	src/client_lbr/friLBRCommand.cpp
	src/client_lbr/friLBRState.cpp
	src/client_trafo/friTransformationClient.cpp
	src/connection/friUdpConnection.cpp
	src/nanopb-0.2.8/pb_decode.c
	src/nanopb-0.2.8/pb_encode.c
	src/protobuf/friCommandMessageEncoder.cpp
	src/protobuf/friMonitoringMessageDecoder.cpp
	src/protobuf/pb_frimessages_callbacks.c
	src/protobuf_gen/FRIMessages.pb.c
 )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
		DIRECTORY include/
		DESTINATION include
)

install(
		TARGETS ${PROJECT_NAME}
		EXPORT export_${PROJECT_NAME}
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include
)

ament_package()
