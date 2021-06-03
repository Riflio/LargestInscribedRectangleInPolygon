QT -= gui
QT += widgets

CONFIG += c++11 console
CONFIG -= app_bundle

SOURCES += \
        geometry.cpp \
        main.cpp

unix {
    INCLUDEPATH += "/usr/local/opencv/include/opencv4/"
    INCLUDEPATH += "/usr/local/include/opencv4/"
    LIBS += -L"/usr/local/opencv/lib/"
    LIBS +=\
        -lopencv_core \
        -lopencv_features2d \
        -lopencv_highgui \
        -lopencv_imgcodecs \
        -lopencv_imgproc \
        -lopencv_video \
        -lopencv_videoio \
        -lopencv_videostab \
        -lopencv_calib3d
}

win32 {
    INCLUDEPATH +="D:/Projects/OpenCV4/opencv/build/install/include"
    LIBS +=-L"D:/Projects/OpenCV4/opencv/build/install/x86/mingw/bin/"
    LIBS +=  \
        -lopencv_core410 \
        -lopencv_highgui410 \
        -lopencv_imgproc410 \
        -lopencv_photo410 \
        -lopencv_imgcodecs410 \
        -lopencv_flann410 \
        -lopencv_features2d410 \
        -lopencv_xfeatures2d410 \
        -lopencv_video410 \
        -lopencv_videoio410 \
        -lopencv_videostab410 \
        -lopencv_bgsegm410 \
        -lopencv_objdetect410 \
        -lopencv_calib3d410



}

HEADERS += \
    geometry.h
