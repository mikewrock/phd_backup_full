/****************************************************************************
** Meta object code from reading C++ file 'shotcretePointsPublisher.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/shotcrete_points/include/shotcrete_selector/shotcretePointsPublisher.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'shotcretePointsPublisher.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rviz_plugin_shotcrete_points_publisher__shotcretePointsPublisher[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      66,   65,   65,   65, 0x0a,
      87,   80,   65,   65, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_rviz_plugin_shotcrete_points_publisher__shotcretePointsPublisher[] = {
    "rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher\0"
    "\0updateTopic()\0pc_msg\0"
    "PointCloudsCallback(sensor_msgs::PointCloud2ConstPtr)\0"
};

void rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        shotcretePointsPublisher *_t = static_cast<shotcretePointsPublisher *>(_o);
        switch (_id) {
        case 0: _t->updateTopic(); break;
        case 1: _t->PointCloudsCallback((*reinterpret_cast< const sensor_msgs::PointCloud2ConstPtr(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::staticMetaObject = {
    { &rviz::SelectionTool::staticMetaObject, qt_meta_stringdata_rviz_plugin_shotcrete_points_publisher__shotcretePointsPublisher,
      qt_meta_data_rviz_plugin_shotcrete_points_publisher__shotcretePointsPublisher, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_plugin_shotcrete_points_publisher__shotcretePointsPublisher))
        return static_cast<void*>(const_cast< shotcretePointsPublisher*>(this));
    typedef rviz::SelectionTool QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int rviz_plugin_shotcrete_points_publisher::shotcretePointsPublisher::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::SelectionTool QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
