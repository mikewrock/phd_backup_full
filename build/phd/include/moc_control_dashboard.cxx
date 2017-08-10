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
      31,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   28,   28,   28, 0x09,
      38,   28,   28,   28, 0x09,
      49,   28,   28,   28, 0x09,
      59,   28,   28,   28, 0x09,
      69,   28,   28,   28, 0x09,
      83,   28,   28,   28, 0x09,
      95,   28,   28,   28, 0x09,
     106,   28,   28,   28, 0x09,
     116,   28,   28,   28, 0x09,
     126,   28,   28,   28, 0x09,
     137,   28,   28,   28, 0x09,
     149,   28,   28,   28, 0x09,
     163,   28,   28,   28, 0x09,
     178,   28,   28,   28, 0x09,
     194,   28,   28,   28, 0x09,
     208,   28,   28,   28, 0x09,
     221,   28,   28,   28, 0x09,
     236,   28,   28,   28, 0x09,
     251,   28,   28,   28, 0x09,
     266,   28,   28,   28, 0x09,
     282,   28,   28,   28, 0x09,
     298,   28,   28,   28, 0x09,
     312,   28,   28,   28, 0x09,
     326,   28,   28,   28, 0x09,
     346,   28,   28,   28, 0x09,
     368,   28,   28,   28, 0x09,
     380,   28,   28,   28, 0x09,
     392,   28,   28,   28, 0x09,
     405,   28,   28,   28, 0x09,
     417,   28,   28,   28, 0x09,
     429,   28,   28,   28, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_control_panel__ControlPanel[] = {
    "control_panel::ControlPanel\0\0do_nav()\0"
    "show_nav()\0exe_nav()\0do_step()\0"
    "do_estimate()\0fake_scan()\0start_pt()\0"
    "fwd_vel()\0rev_vel()\0left_vel()\0"
    "right_vel()\0arm_fwd_vel()\0arm_left_vel()\0"
    "arm_right_vel()\0arm_rev_vel()\0"
    "arm_up_vel()\0arm_down_vel()\0roll_vel_neg()\0"
    "roll_vel_pos()\0pitch_vel_neg()\0"
    "pitch_vel_pos()\0yaw_vel_neg()\0"
    "yaw_vel_pos()\0localization_scan()\0"
    "localization_scan_2()\0clear_vel()\0"
    "cluster_1()\0trajectory()\0cluster_2()\0"
    "cluster_3()\0gen_trajectory()\0"
};

void control_panel::ControlPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ControlPanel *_t = static_cast<ControlPanel *>(_o);
        switch (_id) {
        case 0: _t->do_nav(); break;
        case 1: _t->show_nav(); break;
        case 2: _t->exe_nav(); break;
        case 3: _t->do_step(); break;
        case 4: _t->do_estimate(); break;
        case 5: _t->fake_scan(); break;
        case 6: _t->start_pt(); break;
        case 7: _t->fwd_vel(); break;
        case 8: _t->rev_vel(); break;
        case 9: _t->left_vel(); break;
        case 10: _t->right_vel(); break;
        case 11: _t->arm_fwd_vel(); break;
        case 12: _t->arm_left_vel(); break;
        case 13: _t->arm_right_vel(); break;
        case 14: _t->arm_rev_vel(); break;
        case 15: _t->arm_up_vel(); break;
        case 16: _t->arm_down_vel(); break;
        case 17: _t->roll_vel_neg(); break;
        case 18: _t->roll_vel_pos(); break;
        case 19: _t->pitch_vel_neg(); break;
        case 20: _t->pitch_vel_pos(); break;
        case 21: _t->yaw_vel_neg(); break;
        case 22: _t->yaw_vel_pos(); break;
        case 23: _t->localization_scan(); break;
        case 24: _t->localization_scan_2(); break;
        case 25: _t->clear_vel(); break;
        case 26: _t->cluster_1(); break;
        case 27: _t->trajectory(); break;
        case 28: _t->cluster_2(); break;
        case 29: _t->cluster_3(); break;
        case 30: _t->gen_trajectory(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
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
        if (_id < 31)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 31;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
