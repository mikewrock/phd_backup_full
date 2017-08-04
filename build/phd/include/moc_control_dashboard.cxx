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
      33,   14, // methods
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
      93,   28,   28,   28, 0x09,
     105,   28,   28,   28, 0x09,
     117,   28,   28,   28, 0x09,
     129,   28,   28,   28, 0x09,
     139,   28,   28,   28, 0x09,
     149,   28,   28,   28, 0x09,
     160,   28,   28,   28, 0x09,
     172,   28,   28,   28, 0x09,
     186,   28,   28,   28, 0x09,
     201,   28,   28,   28, 0x09,
     217,   28,   28,   28, 0x09,
     231,   28,   28,   28, 0x09,
     244,   28,   28,   28, 0x09,
     259,   28,   28,   28, 0x09,
     274,   28,   28,   28, 0x09,
     289,   28,   28,   28, 0x09,
     305,   28,   28,   28, 0x09,
     321,   28,   28,   28, 0x09,
     335,   28,   28,   28, 0x09,
     349,   28,   28,   28, 0x09,
     369,   28,   28,   28, 0x09,
     391,   28,   28,   28, 0x09,
     403,   28,   28,   28, 0x09,
     415,   28,   28,   28, 0x09,
     428,   28,   28,   28, 0x09,
     440,   28,   28,   28, 0x09,
     452,   28,   28,   28, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_control_panel__ControlPanel[] = {
    "control_panel::ControlPanel\0\0do_nav()\0"
    "show_nav()\0exe_nav()\0do_step()\0"
    "do_estimate()\0do_scan()\0do_scan_2()\0"
    "do_scan_3()\0do_scan_4()\0fwd_vel()\0"
    "rev_vel()\0left_vel()\0right_vel()\0"
    "arm_fwd_vel()\0arm_left_vel()\0"
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
        case 5: _t->do_scan(); break;
        case 6: _t->do_scan_2(); break;
        case 7: _t->do_scan_3(); break;
        case 8: _t->do_scan_4(); break;
        case 9: _t->fwd_vel(); break;
        case 10: _t->rev_vel(); break;
        case 11: _t->left_vel(); break;
        case 12: _t->right_vel(); break;
        case 13: _t->arm_fwd_vel(); break;
        case 14: _t->arm_left_vel(); break;
        case 15: _t->arm_right_vel(); break;
        case 16: _t->arm_rev_vel(); break;
        case 17: _t->arm_up_vel(); break;
        case 18: _t->arm_down_vel(); break;
        case 19: _t->roll_vel_neg(); break;
        case 20: _t->roll_vel_pos(); break;
        case 21: _t->pitch_vel_neg(); break;
        case 22: _t->pitch_vel_pos(); break;
        case 23: _t->yaw_vel_neg(); break;
        case 24: _t->yaw_vel_pos(); break;
        case 25: _t->localization_scan(); break;
        case 26: _t->localization_scan_2(); break;
        case 27: _t->clear_vel(); break;
        case 28: _t->cluster_1(); break;
        case 29: _t->trajectory(); break;
        case 30: _t->cluster_2(); break;
        case 31: _t->cluster_3(); break;
        case 32: _t->gen_trajectory(); break;
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
        if (_id < 33)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 33;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
