QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    account_manage.cpp \
    database_manager.cpp \
    keyboard.cpp \
    login.cpp \
    main.cpp \
    mainwindow.cpp \
    search_medical.cpp \
    tab_admin.cpp \
    tab_medical.cpp

HEADERS += \
    account_manage.h \
    database_manager.h \
    keyboard.h \
    login.h \
    mainwindow.h \
    search_medical.h \
    tab_admin.h \
    tab_medical.h

FORMS += \
    account_manage.ui \
    keyboard.ui \
    login.ui \
    mainwindow.ui \
    search_medical.ui \
    tab_admin.ui \
    tab_medical.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
