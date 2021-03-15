
set(COM_RQUIRES mcommon mwifi mlink mdebug light_driver light_handle mesh_utils )

#if(CONFIG_LIGHT_EXAMPLE_ALIYUN)
#set(COM_SRCS "util.cpp" "kv.cpp" "main_easypr.cpp" "app_screen.cpp" "nav_light.c" "main.cpp" "system.cpp" "app_camera.c" "aliyun_light_example.c")
#else()
#set(COM_SRCS "light_example.c")
#endif()


#idf_component_register(SRCS ${COM_SRCS}
#                    INCLUDE_DIRS ""
#                    REQUIRES ${COM_REQUIRES}
#)
#}
#)
idf_component_register(
    SRCS
        light_example.c
        main.cpp
        app_screen.cpp
        app_camera.c
        nav_light.c
        system.cpp
        main_easypr.cpp
        util/kv.cpp
        util/util.cpp

        core/chars_identify.cpp
        core/chars_recognise.cpp
        core/chars_segment.cpp
        core/core_func.cpp
        core/feature.cpp
        core/params.cpp
        core/plate_detect.cpp
        core/plate_judge.cpp
        core/plate_locate.cpp
        core/plate_recognize.cpp

        thirdparty/erfilter.cpp
        thirdparty/helper.cpp
        thirdparty/lbp.cpp
        thirdparty/mser2.cpp
        thirdparty/xmlParser.cpp

    INCLUDE_DIRS
        .
        include
        opencv
        include/thirdparty/LBP
        include/thirdparty/mser
        include/thirdparty/svm
        include/thirdparty/textDetect
        include/thirdparty/xmlParser

 #   LDFRAGMENTS 
#        link.lf
)

set(CMAKE_CXX_FLAGS "-Wno-unused-function -Wno-unused-variable ${CMAKE_CXX_FLAGS}")

# Be aware that the order of the librairies is important
add_prebuilt_library(opencv_imgcodecs       "opencv/libopencv_imgcodecs.a")
add_prebuilt_library(libpng                 "opencv/3rdparty/liblibpng.a")
add_prebuilt_library(libzlib                "opencv/3rdparty/libzlib.a")
add_prebuilt_library(opencv_imgproc         "opencv/libopencv_imgproc.a")
add_prebuilt_library(opencv_core            "opencv/libopencv_core.a")
add_prebuilt_library(ade                    "opencv/libade.a")
add_prebuilt_library(opencv_flann           "opencv/libopencv_flann.a")
add_prebuilt_library(opencv_ml              "opencv/libopencv_ml.a")
add_prebuilt_library(pr_thirdparty          "opencv/libthirdparty.a")
add_prebuilt_library(opencv_objdetect       "opencv/libopencv_objdetect.a")


target_link_libraries(${COMPONENT_LIB} PRIVATE ade)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_objdetect)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_imgcodecs)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_imgproc)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_flann)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_ml)
target_link_libraries(${COMPONENT_LIB} PRIVATE opencv_core)
target_link_libraries(${COMPONENT_LIB} PRIVATE libpng)
target_link_libraries(${COMPONENT_LIB} PRIVATE libzlib)
