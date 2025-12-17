/****************************************************************************
** Meta object code from reading C++ file 'wheelchair_admin.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../Hospital_Terminal/wheelchair_admin.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'wheelchair_admin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_wheelchair_admin_t {
    const uint offsetsAndSize[24];
    char stringdata0[206];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_wheelchair_admin_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_wheelchair_admin_t qt_meta_stringdata_wheelchair_admin = {
    {
QT_MOC_LITERAL(0, 16), // "wheelchair_admin"
QT_MOC_LITERAL(17, 28), // "on_twRobotStatus_cellClicked"
QT_MOC_LITERAL(46, 0), // ""
QT_MOC_LITERAL(47, 3), // "row"
QT_MOC_LITERAL(51, 6), // "column"
QT_MOC_LITERAL(58, 23), // "on_pbDirectCall_clicked"
QT_MOC_LITERAL(82, 17), // "on_pbStop_clicked"
QT_MOC_LITERAL(100, 19), // "on_pbResume_clicked"
QT_MOC_LITERAL(120, 19), // "on_pbGoWait_clicked"
QT_MOC_LITERAL(140, 21), // "on_pbGoCharge_clicked"
QT_MOC_LITERAL(162, 21), // "on_pbAddWheel_clicked"
QT_MOC_LITERAL(184, 21) // "on_pushButton_clicked"

    },
    "wheelchair_admin\0on_twRobotStatus_cellClicked\0"
    "\0row\0column\0on_pbDirectCall_clicked\0"
    "on_pbStop_clicked\0on_pbResume_clicked\0"
    "on_pbGoWait_clicked\0on_pbGoCharge_clicked\0"
    "on_pbAddWheel_clicked\0on_pushButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_wheelchair_admin[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   62,    2, 0x08,    1 /* Private */,
       5,    0,   67,    2, 0x08,    4 /* Private */,
       6,    0,   68,    2, 0x08,    5 /* Private */,
       7,    0,   69,    2, 0x08,    6 /* Private */,
       8,    0,   70,    2, 0x08,    7 /* Private */,
       9,    0,   71,    2, 0x08,    8 /* Private */,
      10,    0,   72,    2, 0x08,    9 /* Private */,
      11,    0,   73,    2, 0x08,   10 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void wheelchair_admin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<wheelchair_admin *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_twRobotStatus_cellClicked((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 1: _t->on_pbDirectCall_clicked(); break;
        case 2: _t->on_pbStop_clicked(); break;
        case 3: _t->on_pbResume_clicked(); break;
        case 4: _t->on_pbGoWait_clicked(); break;
        case 5: _t->on_pbGoCharge_clicked(); break;
        case 6: _t->on_pbAddWheel_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject wheelchair_admin::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_wheelchair_admin.offsetsAndSize,
    qt_meta_data_wheelchair_admin,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_wheelchair_admin_t
, QtPrivate::TypeAndForceComplete<wheelchair_admin, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *wheelchair_admin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *wheelchair_admin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_wheelchair_admin.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int wheelchair_admin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
