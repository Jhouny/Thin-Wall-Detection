/****************************************************************************
** Meta object code from reading C++ file 'GraphicsViewInput.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../usr/include/CGAL/Qt/GraphicsViewInput.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GraphicsViewInput.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CGAL__Qt__GraphicsViewInput_t {
    QByteArrayData data[7];
    char stringdata0[79];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__Qt__GraphicsViewInput_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__Qt__GraphicsViewInput_t qt_meta_stringdata_CGAL__Qt__GraphicsViewInput = {
    {
QT_MOC_LITERAL(0, 0, 27), // "CGAL::Qt::GraphicsViewInput"
QT_MOC_LITERAL(1, 28, 8), // "generate"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 12), // "CGAL::Object"
QT_MOC_LITERAL(4, 51, 1), // "o"
QT_MOC_LITERAL(5, 53, 12), // "modelChanged"
QT_MOC_LITERAL(6, 66, 12) // "processInput"

    },
    "CGAL::Qt::GraphicsViewInput\0generate\0"
    "\0CGAL::Object\0o\0modelChanged\0processInput"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__Qt__GraphicsViewInput[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       5,    0,   32,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   33,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    2,

       0        // eod
};

void CGAL::Qt::GraphicsViewInput::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GraphicsViewInput *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->generate((*reinterpret_cast< CGAL::Object(*)>(_a[1]))); break;
        case 1: _t->modelChanged(); break;
        case 2: _t->processInput((*reinterpret_cast< CGAL::Object(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (GraphicsViewInput::*)(CGAL::Object );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GraphicsViewInput::generate)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (GraphicsViewInput::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GraphicsViewInput::modelChanged)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CGAL::Qt::GraphicsViewInput::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CGAL__Qt__GraphicsViewInput.data,
    qt_meta_data_CGAL__Qt__GraphicsViewInput,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CGAL::Qt::GraphicsViewInput::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::Qt::GraphicsViewInput::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__Qt__GraphicsViewInput.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int CGAL::Qt::GraphicsViewInput::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void CGAL::Qt::GraphicsViewInput::generate(CGAL::Object _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CGAL::Qt::GraphicsViewInput::modelChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
