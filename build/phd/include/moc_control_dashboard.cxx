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
      34,   14, // methods
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
      75,   28,   28,   28, 0x09,
      86,   28,   28,   28, 0x09,
      98,   28,   28,   28, 0x09,
     111,   28,   28,   28, 0x09,
     118,   28,   28,   28, 0x09,
     128,   28,   28,   28, 0x09,
     139,   28,   28,   28, 0x09,
     153,   28,   28,   28, 0x09,
     163,   28,   28,   28, 0x09,
     180,   28,   28,   28, 0x09,
     195,   28,   28,   28, 0x09,
     209,   28,   28,   28, 0x09,
     222,   28,   28,   28, 0x09,
     239,   28,   28,   28, 0x09,
     257,   28,   28,   28, 0x09,
     277,   28,   28,   28, 0x09,
     294,   28,   28,   28, 0x09,
     306,   28,   28,   28, 0x09,
     322,   28,   28,   28, 0x09,
     338,   28,   28,   28, 0x09,
     351,   28,   28,   28, 0x09,
     363,   28,   28,   28, 0x09,
     375,   28,   28,   28, 0x09,
     387,   28,   28,   28, 0x09,
     399,   28,   28,   28, 0x09,
     411,   28,   28,   28, 0x09,
     425,   28,   28,   28, 0x09,
     447,   28,   28,   28, 0x09,
     469,   28,   28,   28, 0x09,
     500,  492,  486,   28, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_control_panel__ControlPanel[] = {
    "control_panel::ControlPanel\0\0do_nav()\0"
    "show_nav()\0arm_step()\0arm_step_man()\0"
    "arm_loop()\0fake_scan()\0fake_scan2()\0"
    "scan()\0prescan()\0postscan()\0homing_scan()\0"
    "cluster()\0gen_trajectory()\0onPTPCommand()\0"
    "onCPCommand()\0onTCommand()\0onJointCommand()\0"
    "onStringCommand()\0onShutdownCommand()\0"
    "onSpeedCommand()\0load_traj()\0"
    "print_markers()\0clear_markers()\0"
    "rosSpinner()\0soft_stop()\0pose_step()\0"
    "pose_loop()\0reset_map()\0set_clean()\0"
    "set_sprayed()\0calculate_thickness()\0"
    "thickness_from_file()\0save_selection()\0"
    "float\0arm,tgt\0"
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
        case 3: _t->arm_step_man(); break;
        case 4: _t->arm_loop(); break;
        case 5: _t->fake_scan(); break;
        case 6: _t->fake_scan2(); break;
        case 7: _t->scan(); break;
        case 8: _t->prescan(); break;
        case 9: _t->postscan(); break;
        case 10: _t->homing_scan(); break;
        case 11: _t->cluster(); break;
        case 12: _t->gen_trajectory(); break;
        case 13: _t->onPTPCommand(); break;
        case 14: _t->onCPCommand(); break;
        case 15: _t->onTCommand(); break;
        case 16: _t->onJointCommand(); break;
        case 17: _t->onStringCommand(); break;
        case 18: _t->onShutdownCommand(); break;
        case 19: _t->onSpeedCommand(); break;
        case 20: _t->load_traj(); break;
        case 21: _t->print_markers(); break;
        case 22: _t->clear_markers(); break;
        case 23: _t->rosSpinner(); break;
        case 24: _t->soft_stop(); break;
        case 25: _t->pose_step(); break;
        case 26: _t->pose_loop(); break;
        case 27: _t->reset_map(); break;
        case 28: _t->set_clean(); break;
        case 29: _t->set_sprayed(); break;
        case 30: _t->calculate_thickness(); break;
        case 31: _t->thickness_from_file(); break;
        case 32: _t->save_selection(); break;
        case 33: { float _r = _t->tgt_distance((*reinterpret_cast< phd::arm_msg(*)>(_a[1])),(*reinterpret_cast< phd::trajectory_point(*)>(_a[2])));
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
        if (_id < 34)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 34;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
