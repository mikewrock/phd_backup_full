/****************************************************************************
** Meta object code from reading C++ file 'dashboard_panel.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/denso_gui/include/denso_gui/dashboard_panel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dashboard_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_denso_gui__DensoDashboardPanel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      32,   31,   31,   31, 0x09,
      47,   31,   31,   31, 0x09,
      62,   31,   31,   31, 0x09,
      76,   31,   31,   31, 0x09,
      96,   31,   31,   31, 0x09,
     118,   31,   31,   31, 0x09,
     135,   31,   31,   31, 0x09,
     153,   31,   31,   31, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_denso_gui__DensoDashboardPanel[] = {
    "denso_gui::DensoDashboardPanel\0\0"
    "onLaunchNode()\0onPTPCommand()\0"
    "onCPCommand()\0onShutdownCommand()\0"
    "onClearErrorCommand()\0onJointCommand()\0"
    "onStringCommand()\0onSpeedCommand()\0"
};

void denso_gui::DensoDashboardPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        DensoDashboardPanel *_t = static_cast<DensoDashboardPanel *>(_o);
        switch (_id) {
        case 0: _t->onLaunchNode(); break;
        case 1: _t->onPTPCommand(); break;
        case 2: _t->onCPCommand(); break;
        case 3: _t->onShutdownCommand(); break;
        case 4: _t->onClearErrorCommand(); break;
        case 5: _t->onJointCommand(); break;
        case 6: _t->onStringCommand(); break;
        case 7: _t->onSpeedCommand(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData denso_gui::DensoDashboardPanel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject denso_gui::DensoDashboardPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_denso_gui__DensoDashboardPanel,
      qt_meta_data_denso_gui__DensoDashboardPanel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &denso_gui::DensoDashboardPanel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *denso_gui::DensoDashboardPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *denso_gui::DensoDashboardPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_denso_gui__DensoDashboardPanel))
        return static_cast<void*>(const_cast< DensoDashboardPanel*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int denso_gui::DensoDashboardPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
