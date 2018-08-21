/****************************************************************************
** Meta object code from reading C++ file 'control_dashboard.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/phd/include/control_dashboard.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'control_dashboard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_control_panel__ControlPanel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      29,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   28,   28,   28, 0x09,
      38,   28,   28,   28, 0x09,
      49,   28,   28,   28, 0x09,
      60,   28,   28,   28, 0x09,
      71,   28,   28,   28, 0x09,
      83,   28,   28,   28, 0x09,
      96,   28,   28,   28, 0x09,
     103,   28,   28,   28, 0x09,
     117,   28,   28,   28, 0x09,
     127,   28,   28,   28, 0x09,
     144,   28,   28,   28, 0x09,
     159,   28,   28,   28, 0x09,
     173,   28,   28,   28, 0x09,
     186,   28,   28,   28, 0x09,
     203,   28,   28,   28, 0x09,
     221,   28,   28,   28, 0x09,
     241,   28,   28,   28, 0x09,
     258,   28,   28,   28, 0x09,
     270,   28,   28,   28, 0x09,
     283,   28,   28,   28, 0x09,
     295,   28,   28,   28, 0x09,
     307,   28,   28,   28, 0x09,
     319,   28,   28,   28, 0x09,
     331,   28,   28,   28, 0x09,
     343,   28,   28,   28, 0x09,
     357,   28,   28,   28, 0x09,
     379,   28,   28,   28, 0x09,
     401,   28,   28,   28, 0x09,
     432,  424,  418,   28, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_control_panel__ControlPanel[] = {
    "control_panel::ControlPanel\0\0do_nav()\0"
    "show_nav()\0arm_step()\0arm_loop()\0"
    "fake_scan()\0fake_scan2()\0scan()\0"
    "homing_scan()\0cluster()\0gen_trajectory()\0"
    "onPTPCommand()\0onCPCommand()\0onTCommand()\0"
    "onJointCommand()\0onStringCommand()\0"
    "onShutdownCommand()\0onSpeedCommand()\0"
    "load_traj()\0rosSpinner()\0soft_stop()\0"
    "pose_step()\0pose_loop()\0reset_map()\0"
    "set_clean()\0set_sprayed()\0"
    "calculate_thickness()\0thickness_from_file()\0"
    "save_selection()\0float\0arm,tgt\0"
    "tgt_distance(phd::arm_msg,phd::trajectory_point)\0"
};

void control_panel::ControlPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ControlPanel *_t = static_cast<ControlPanel *>(_o);
        switch (_id) {
        case 0: _t->do_nav(); break;
        case 1: _t->show_nav(); break;
        case 2: _t->arm_step(); break;
        case 3: _t->arm_loop(); break;
        case 4: _t->fake_scan(); break;
        case 5: _t->fake_scan2(); break;
        case 6: _t->scan(); break;
        case 7: _t->homing_scan(); break;
        case 8: _t->cluster(); break;
        case 9: _t->gen_trajectory(); break;
        case 10: _t->onPTPCommand(); break;
        case 11: _t->onCPCommand(); break;
        case 12: _t->onTCommand(); break;
        case 13: _t->onJointCommand(); break;
        case 14: _t->onStringCommand(); break;
        case 15: _t->onShutdownCommand(); break;
        case 16: _t->onSpeedCommand(); break;
        case 17: _t->load_traj(); break;
        case 18: _t->rosSpinner(); break;
        case 19: _t->soft_stop(); break;
        case 20: _t->pose_step(); break;
        case 21: _t->pose_loop(); break;
        case 22: _t->reset_map(); break;
        case 23: _t->set_clean(); break;
        case 24: _t->set_sprayed(); break;
        case 25: _t->calculate_thickness(); break;
        case 26: _t->thickness_from_file(); break;
        case 27: _t->save_selection(); break;
        case 28: { float _r = _t->tgt_distance((*reinterpret_cast< phd::arm_msg(*)>(_a[1])),(*reinterpret_cast< phd::trajectory_point(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData control_panel::ControlPanel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject control_panel::ControlPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_control_panel__ControlPanel,
      qt_meta_data_control_panel__ControlPanel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &control_panel::ControlPanel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *control_panel::ControlPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *control_panel::ControlPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_control_panel__ControlPanel))
        return static_cast<void*>(const_cast< ControlPanel*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int control_panel::ControlPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 29)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 29;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
