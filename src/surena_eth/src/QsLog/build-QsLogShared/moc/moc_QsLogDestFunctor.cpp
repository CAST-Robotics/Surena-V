/****************************************************************************
** Meta object code from reading C++ file 'QsLogDestFunctor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../QsLogDestFunctor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QsLogDestFunctor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_QsLogging__FunctorDestination_t {
    QByteArrayData data[5];
    char stringdata0[61];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QsLogging__FunctorDestination_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QsLogging__FunctorDestination_t qt_meta_stringdata_QsLogging__FunctorDestination = {
    {
QT_MOC_LITERAL(0, 0, 29), // "QsLogging::FunctorDestination"
QT_MOC_LITERAL(1, 30, 15), // "logMessageReady"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 7), // "message"
QT_MOC_LITERAL(4, 55, 5) // "level"

    },
    "QsLogging::FunctorDestination\0"
    "logMessageReady\0\0message\0level"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QsLogging__FunctorDestination[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   19,    2, 0x05 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    3,    4,

       0        // eod
};

void QsLogging::FunctorDestination::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FunctorDestination *_t = static_cast<FunctorDestination *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->logMessageReady((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (FunctorDestination::*_t)(const QString & , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&FunctorDestination::logMessageReady)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject QsLogging::FunctorDestination::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_QsLogging__FunctorDestination.data,
      qt_meta_data_QsLogging__FunctorDestination,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QsLogging::FunctorDestination::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QsLogging::FunctorDestination::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QsLogging__FunctorDestination.stringdata0))
        return static_cast<void*>(const_cast< FunctorDestination*>(this));
    if (!strcmp(_clname, "Destination"))
        return static_cast< Destination*>(const_cast< FunctorDestination*>(this));
    return QObject::qt_metacast(_clname);
}

int QsLogging::FunctorDestination::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void QsLogging::FunctorDestination::logMessageReady(const QString & _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
