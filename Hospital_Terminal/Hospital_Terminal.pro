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
    tab_admin.cpp

HEADERS += \
    account_manage.h \
    database_manager.h \
    keyboard.h \
    login.h \
    mainwindow.h \
    tab_admin.h

FORMS += \
    account_manage.ui \
    keyboard.ui \
    login.ui \
    mainwindow.ui \
    tab_admin.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
