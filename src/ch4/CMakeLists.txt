# 查找 Google Test，它应该也包含了 Google Mock
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
# 添加您的库
add_library(${PROJECT_NAME}.ch4
        gins_pre_integ.cc
        imu_preintegration.cc
        g2o_types.cc
)
 
# 添加您的测试可执行文件
add_executable(test_preintegration test_preintegration.cc)
 
# 链接 Google Test (它应该包含了 gmock) 以及其他依赖项
target_link_libraries(test_preintegration
        ${GTEST_BOTH_LIBRARIES}
        pthread glog gflags ${PROJECT_NAME}.ch4 ${PROJECT_NAME}.ch3 ${PROJECT_NAME}.common
)
 
# 添加测试
add_test(NAME test_preintegration COMMAND test_preintegration)
 
# 添加其他可执行文件
add_executable(run_gins_pre_integ run_gins_pre_integ.cc)
target_link_libraries(run_gins_pre_integ
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch4
        ${G2O_LIBS}
	)
