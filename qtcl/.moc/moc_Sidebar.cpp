/****************************************************************************
** Meta object code from reading C++ file 'Sidebar.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../Sidebar.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Sidebar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CQTclModel3DView__Sidebar_t {
    QByteArrayData data[7];
    char stringdata0[81];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CQTclModel3DView__Sidebar_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CQTclModel3DView__Sidebar_t qt_meta_stringdata_CQTclModel3DView__Sidebar = {
    {
QT_MOC_LITERAL(0, 0, 25), // "CQTclModel3DView::Sidebar"
QT_MOC_LITERAL(1, 26, 10), // "selectSlot"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 5), // "state"
QT_MOC_LITERAL(4, 44, 10), // "cameraSlot"
QT_MOC_LITERAL(5, 55, 7), // "tclSlot"
QT_MOC_LITERAL(6, 63, 17) // "updateButtonState"

    },
    "CQTclModel3DView::Sidebar\0selectSlot\0"
    "\0state\0cameraSlot\0tclSlot\0updateButtonState"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CQTclModel3DView__Sidebar[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x08 /* Private */,
       4,    1,   37,    2, 0x08 /* Private */,
       5,    1,   40,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,

       0        // eod
};

void CQTclModel3DView::Sidebar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Sidebar *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->selectSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->cameraSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->tclSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->updateButtonState(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CQTclModel3DView::Sidebar::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_CQTclModel3DView__Sidebar.data,
    qt_meta_data_CQTclModel3DView__Sidebar,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CQTclModel3DView::Sidebar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CQTclModel3DView::Sidebar::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CQTclModel3DView__Sidebar.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int CQTclModel3DView::Sidebar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
