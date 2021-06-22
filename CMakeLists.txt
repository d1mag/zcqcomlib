cmake_minimum_required(VERSION 3.11)
project(zcqcomlib VERSION 1.0.0 DESCRIPTION "Zeno CANquatro communication library" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (UNIX)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    
    if(CMAKE_BUILD_TYPE MATCHES Debug)
      set(CMAKE_CXX_FLAGS -pg)
      set(CMAKE_EXE_LINKER_FLAGS -pg)
      set(CMAKE_SHARED_LINKER_FLAGS -pg)
    endif(CMAKE_BUILD_TYPE MATCHES Debug)
endif (UNIX)

if (WIN32)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    set(LIBUSB_INCLUDE_DIRS "C:\\proj\\libusb")

    message("build type ${CMAKE_BUILD_TYPE}")
    IF(CMAKE_BUILD_TYPE MATCHES Debug)
        message("debug mode")
        set(LIBUSB_LIBRARIES "C:\\proj\\libusb\\x64\\Debug\\lib\\libusb-1.0.lib")
    ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
        set(LIBUSB_LIBRARIES "C:\\proj\\libusb\\x64\\Release\\lib\\libusb-1.0.lib")
    ELSE()
        message("defaulint to release mode")
        set(LIBUSB_LIBRARIES "C:\\proj\\libusb\\x64\\Release\\lib\\libusb-1.0.lib")
    ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

endif (WIN32)

set(SOURCES
  src/canlib.cpp
  src/linlib.cpp
  src/zdebug.cpp
  src/zcqcore.cpp
  src/PCANBasic.cpp
  src/zusbcontext.cpp
  src/zusbeventthread.cpp
  src/zzenocanchannel.cpp
  src/zzenocandriver.cpp
  src/zzenolinchannel.cpp
  src/zzenolindriver.cpp
  src/zzenousbdevice.cpp
  src/zthreadlocalstring.cpp
  src/zzenotimersynch.cpp
)

set(HEADERS
  include/canlib.h
  include/canstat.h
  include/obsolete.h
  include/linlib.h
  include/vcanevt.h
  include/zpushpack.h
  include/zpoppack.h
  include/PCANBasic.h
  src/zglobal.h
  src/zdebug.h
  src/zring.h
  src/zcqcore.h  
  src/zrefcountingobjbase.h
  src/zcanchannel.h
  src/zcandriver.h
  src/zlinchannel.h
  src/zlindriver.h
  src/zenocan.h
  src/zusbeventthread.h
  src/zzenocanchannel.h
  src/zzenolinchannel.h
  src/zzenousbdevice.h
  src/zcandriverfactory.h
  src/zcanflags.h
  src/zusbcontext.h
  src/zusb.h
  src/zzenocandriver.h
  src/zzenolindriver.h
  src/zthreadlocalstring.h
  src/zzenotimersynch.h
)

set(INLCUDE_DIRS
    ${LIBUSB_INCLUDE_DIRS}
    include
)

add_library(zcqcomlib SHARED ${SOURCES} ${HEADERS} )

target_link_libraries(zcqcomlib ${LIBUSB_LIBRARIES} -lpthread)
target_include_directories(zcqcomlib PUBLIC ${INLCUDE_DIRS})
target_compile_options(zcqcomlib PUBLIC ${LIBUSB_CFLAGS_OTHER} -pthread)

add_subdirectory(examples)
