QT -= gui
QT += widgets

CONFIG += c++11 console
CONFIG -= app_bundle

SOURCES += \
        inscribedmaxarearect.cpp \
        main.cpp

unix {
    INCLUDEPATH += "/usr/local/opencv/include/opencv4/"
    INCLUDEPATH += "/usr/local/include/opencv4/"
    LIBS += -L"/usr/local/opencv/lib/"
    LIBS +=\
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \

}

win32 {
    INCLUDEPATH +="D:/Projects/OpenCV4/opencv/build/install/include"
    LIBS +=-L"D:/Projects/OpenCV4/opencv/build/install/x86/mingw/bin/"
    LIBS +=  \
        -lopencv_core410 \
        -lopencv_highgui410 \
        -lopencv_imgproc410 \


}

HEADERS += \
    earcut.hpp/include/mapbox/earcut.hpp \
    geometry.h \
    inscribedmaxarearect.h
