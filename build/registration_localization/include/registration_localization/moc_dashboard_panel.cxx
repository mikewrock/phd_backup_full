/****************************************************************************
** Meta object code from reading C++ file 'dashboard_panel.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/registration_localization/include/registration_localization/dashboard_panel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dashboard_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_registration_localization__DashboardPanel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      43,   42,   42,   42, 0x09,
      58,   42,   42,   42, 0x09,
      73,   42,   42,   42, 0x09,
      88,   42,   42,   42, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_registration_localization__DashboardPanel[] = {
    "registration_localization::DashboardPanel\0"
    "\0onLaunchNode()\0onLoadMarker()\0"
    "onShowMarker()\0onAlign()\0"
};

void registration_localization::DashboardPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        DashboardPanel *_t = static_cast<DashboardPanel *>(_o);
        switch (_id) {
        case 0: _t->onLaunchNode(); break;
        case 1: _t->onLoadMarker(); break;
        case 2: _t->onShowMarker(); break;
        case 3: _t->onAlign(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData registration_localization::DashboardPanel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject registration_localization::DashboardPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_registration_localization__DashboardPanel,
      qt_meta_data_registration_localization__DashboardPanel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &registration_localization::DashboardPanel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *registration_localization::DashboardPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *registration_localization::DashboardPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_registration_localization__DashboardPanel))
        return static_cast<void*>(const_cast< DashboardPanel*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int registration_localization::DashboardPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
