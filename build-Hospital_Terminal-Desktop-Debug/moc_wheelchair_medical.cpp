/****************************************************************************
** Meta object code from reading C++ file 'wheelchair_medical.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../Hospital_Terminal/wheelchair_medical.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'wheelchair_medical.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_wheelchair_medical_t {
    const uint offsetsAndSize[16];
    char stringdata0[120];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_wheelchair_medical_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_wheelchair_medical_t qt_meta_stringdata_wheelchair_medical = {
    {
QT_MOC_LITERAL(0, 18), // "wheelchair_medical"
QT_MOC_LITERAL(19, 27), // "on_pbSendWheelchair_clicked"
QT_MOC_LITERAL(47, 0), // ""
QT_MOC_LITERAL(48, 25), // "on_cbBoarded_stateChanged"
QT_MOC_LITERAL(74, 5), // "state"
QT_MOC_LITERAL(80, 28), // "on_twReservation_cellClicked"
QT_MOC_LITERAL(109, 3), // "row"
QT_MOC_LITERAL(113, 6) // "column"

    },
    "wheelchair_medical\0on_pbSendWheelchair_clicked\0"
    "\0on_cbBoarded_stateChanged\0state\0"
    "on_twReservation_cellClicked\0row\0"
    "column"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_wheelchair_medical[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   32,    2, 0x08,    1 /* Private */,
       3,    1,   33,    2, 0x08,    2 /* Private */,
       5,    2,   36,    2, 0x08,    4 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    6,    7,

       0        // eod
};

void wheelchair_medical::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<wheelchair_medical *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_pbSendWheelchair_clicked(); break;
        case 1: _t->on_cbBoarded_stateChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 2: _t->on_twReservation_cellClicked((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObject wheelchair_medical::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_wheelchair_medical.offsetsAndSize,
    qt_meta_data_wheelchair_medical,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_wheelchair_medical_t
, QtPrivate::TypeAndForceComplete<wheelchair_medical, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>


>,
    nullptr
} };


const QMetaObject *wheelchair_medical::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *wheelchair_medical::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_wheelchair_medical.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int wheelchair_medical::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
