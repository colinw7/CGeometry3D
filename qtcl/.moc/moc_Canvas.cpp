/****************************************************************************
** Meta object code from reading C++ file 'Canvas.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../Canvas.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Canvas.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CQTclModel3DView__Canvas_t {
    QByteArrayData data[9];
    char stringdata0[128];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CQTclModel3DView__Canvas_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CQTclModel3DView__Canvas_t qt_meta_stringdata_CQTclModel3DView__Canvas = {
    {
QT_MOC_LITERAL(0, 0, 24), // "CQTclModel3DView::Canvas"
QT_MOC_LITERAL(1, 25, 14), // "glStateChanged"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 17), // "selectTypeChanged"
QT_MOC_LITERAL(4, 59, 15), // "editTypeChanged"
QT_MOC_LITERAL(5, 75, 12), // "textureAdded"
QT_MOC_LITERAL(6, 88, 16), // "selectionChanged"
QT_MOC_LITERAL(7, 105, 11), // "updateScene"
QT_MOC_LITERAL(8, 117, 10) // "updateBBox"

    },
    "CQTclModel3DView::Canvas\0glStateChanged\0"
    "\0selectTypeChanged\0editTypeChanged\0"
    "textureAdded\0selectionChanged\0updateScene\0"
    "updateBBox"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CQTclModel3DView__Canvas[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    0,   51,    2, 0x06 /* Public */,
       5,    0,   52,    2, 0x06 /* Public */,
       6,    0,   53,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   54,    2, 0x0a /* Public */,
       7,    0,   57,    2, 0x2a /* Public | MethodCloned */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void,

       0        // eod
};

void CQTclModel3DView::Canvas::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Canvas *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->glStateChanged(); break;
        case 1: _t->selectTypeChanged(); break;
        case 2: _t->editTypeChanged(); break;
        case 3: _t->textureAdded(); break;
        case 4: _t->selectionChanged(); break;
        case 5: _t->updateScene((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->updateScene(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Canvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Canvas::glStateChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Canvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Canvas::selectTypeChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Canvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Canvas::editTypeChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Canvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Canvas::textureAdded)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (Canvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Canvas::selectionChanged)) {
                *result = 4;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CQTclModel3DView::Canvas::staticMetaObject = { {
    QMetaObject::SuperData::link<QGLWidget::staticMetaObject>(),
    qt_meta_stringdata_CQTclModel3DView__Canvas.data,
    qt_meta_data_CQTclModel3DView__Canvas,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CQTclModel3DView::Canvas::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CQTclModel3DView::Canvas::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CQTclModel3DView__Canvas.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QOpenGLExtraFunctions"))
        return static_cast< QOpenGLExtraFunctions*>(this);
    return QGLWidget::qt_metacast(_clname);
}

int CQTclModel3DView::Canvas::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void CQTclModel3DView::Canvas::glStateChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void CQTclModel3DView::Canvas::selectTypeChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void CQTclModel3DView::Canvas::editTypeChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void CQTclModel3DView::Canvas::textureAdded()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void CQTclModel3DView::Canvas::selectionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
