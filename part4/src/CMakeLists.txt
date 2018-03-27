#cmake_minimum_required(VERSION 2.8)
#project(generate_pointcloud)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-std=c++11 -03")

# 增加一個可執行二進制文件
ADD_EXECUTABLE(main main.cpp)
# 增加PCL庫依賴
FIND_PACKAGE(PCL REQUIRED)
# 增加eigen3頭文件目錄
include_directories("/usr/include/eigen3")

# 增加opencv的依賴
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRES})

# 添加頭文件和庫文件
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

#ADD_EXECUTABLE(generate_pointcloud1 generatePointCloud.cpp)
ADD_EXECUTABLE(generate_pointcloud1 generatePointCloud1.cpp)
TARGET_LINK_LIBRARIES(generate_pointcloud1 ${OpenCV_LIBS}
	${PCL_LIBRARIES})

ADD_LIBRARY(slamBase slamBase.cpp)
TARGET_LINK_LIBRARIES(slamBase
	${OpenCV_LIBS}
	${PCL_LIBRARIES})

ADD_EXECUTABLE(detectFeatures detectFeatures.cpp)
TARGET_LINK_LIBRARIES(detectFeatures slamBase
			${OpenCV_LIBS}
			${PCL_LIBRARIES})

ADD_EXECUTABLE(jointPointCloud jointPointCloud.cpp)
TARGET_LINK_LIBRARIES(jointPointCloud slamBase
			${OpenCV_LIBS}
			${PCL_LIBRARIES})



