/****************************************************************************
** Meta object code from reading C++ file 'Toolbar.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../Toolbar.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Toolbar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CQTclModel3DView__Toolbar_t {
    QByteArrayData data[16];
    char stringdata0[216];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CQTclModel3DView__Toolbar_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CQTclModel3DView__Toolbar_t qt_meta_stringdata_CQTclModel3DView__Toolbar = {
    {
QT_MOC_LITERAL(0, 0, 25), // "CQTclModel3DView::Toolbar"
QT_MOC_LITERAL(1, 26, 9), // "pauseSlot"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 16), // "objectSelectSlot"
QT_MOC_LITERAL(4, 54, 14), // "faceSelectSlot"
QT_MOC_LITERAL(5, 69, 14), // "edgeSelectSlot"
QT_MOC_LITERAL(6, 84, 15), // "pointSelectSlot"
QT_MOC_LITERAL(7, 100, 14), // "selectNoneSlot"
QT_MOC_LITERAL(8, 115, 13), // "depthTestSlot"
QT_MOC_LITERAL(9, 129, 1), // "b"
QT_MOC_LITERAL(10, 131, 12), // "cullFaceSlot"
QT_MOC_LITERAL(11, 144, 13), // "frontFaceSlot"
QT_MOC_LITERAL(12, 158, 13), // "wireframeSlot"
QT_MOC_LITERAL(13, 172, 13), // "solidFillSlot"
QT_MOC_LITERAL(14, 186, 15), // "textureFillSlot"
QT_MOC_LITERAL(15, 202, 13) // "updateWidgets"

    },
    "CQTclModel3DView::Toolbar\0pauseSlot\0"
    "\0objectSelectSlot\0faceSelectSlot\0"
    "edgeSelectSlot\0pointSelectSlot\0"
    "selectNoneSlot\0depthTestSlot\0b\0"
    "cullFaceSlot\0frontFaceSlot\0wireframeSlot\0"
    "solidFillSlot\0textureFillSlot\0"
    "updateWidgets"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CQTclModel3DView__Toolbar[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x08 /* Private */,
       3,    1,   82,    2, 0x08 /* Private */,
       4,    1,   85,    2, 0x08 /* Private */,
       5,    1,   88,    2, 0x08 /* Private */,
       6,    1,   91,    2, 0x08 /* Private */,
       7,    0,   94,    2, 0x08 /* Private */,
       8,    1,   95,    2, 0x08 /* Private */,
      10,    1,   98,    2, 0x08 /* Private */,
      11,    1,  101,    2, 0x08 /* Private */,
      12,    0,  104,    2, 0x08 /* Private */,
      13,    0,  105,    2, 0x08 /* Private */,
      14,    0,  106,    2, 0x08 /* Private */,
      15,    0,  107,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CQTclModel3DView::Toolbar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Toolbar *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->pauseSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->objectSelectSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->faceSelectSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->edgeSelectSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->pointSelectSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->selectNoneSlot(); break;
        case 6: _t->depthTestSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->cullFaceSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->frontFaceSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->wireframeSlot(); break;
        case 10: _t->solidFillSlot(); break;
        case 11: _t->textureFillSlot(); break;
        case 12: _t->updateWidgets(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CQTclModel3DView::Toolbar::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_CQTclModel3DView__Toolbar.data,
    qt_meta_data_CQTclModel3DView__Toolbar,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CQTclModel3DView::Toolbar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CQTclModel3DView::Toolbar::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CQTclModel3DView__Toolbar.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int CQTclModel3DView::Toolbar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
