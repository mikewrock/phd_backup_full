//#line 2 "/opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the phd package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

#ifndef __phd__TRAJECTORYCONFIG_H__
#define __phd__TRAJECTORYCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace phd
{
  class TrajectoryConfigStatics;
  
  class TrajectoryConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(TrajectoryConfig &config, const TrajectoryConfig &max, const TrajectoryConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const TrajectoryConfig &config1, const TrajectoryConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, TrajectoryConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const TrajectoryConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, TrajectoryConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const TrajectoryConfig &config) const = 0;
      virtual void getValue(const TrajectoryConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T TrajectoryConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (TrajectoryConfig::* field);

      virtual void clamp(TrajectoryConfig &config, const TrajectoryConfig &max, const TrajectoryConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const TrajectoryConfig &config1, const TrajectoryConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, TrajectoryConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const TrajectoryConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, TrajectoryConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const TrajectoryConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const TrajectoryConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, TrajectoryConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, TrajectoryConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<TrajectoryConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(TrajectoryConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("workspace"==(*_i)->name){workspace = boost::any_cast<double>(val);}
        if("height_step"==(*_i)->name){height_step = boost::any_cast<double>(val);}
        if("max_height"==(*_i)->name){max_height = boost::any_cast<double>(val);}
        if("mid_height"==(*_i)->name){mid_height = boost::any_cast<double>(val);}
        if("offset"==(*_i)->name){offset = boost::any_cast<double>(val);}
        if("slice_thickness"==(*_i)->name){slice_thickness = boost::any_cast<double>(val);}
        if("via_distance"==(*_i)->name){via_distance = boost::any_cast<double>(val);}
        if("radial_steps"==(*_i)->name){radial_steps = boost::any_cast<double>(val);}
        if("start_angle"==(*_i)->name){start_angle = boost::any_cast<double>(val);}
        if("end_angle"==(*_i)->name){end_angle = boost::any_cast<double>(val);}
        if("rib_downsampling"==(*_i)->name){rib_downsampling = boost::any_cast<double>(val);}
        if("rib_spacing"==(*_i)->name){rib_spacing = boost::any_cast<double>(val);}
        if("z_vert_limit"==(*_i)->name){z_vert_limit = boost::any_cast<double>(val);}
        if("rib_tolerance"==(*_i)->name){rib_tolerance = boost::any_cast<double>(val);}
        if("max_rib_length"==(*_i)->name){max_rib_length = boost::any_cast<double>(val);}
      }
    }

    double workspace;
double height_step;
double max_height;
double mid_height;
double offset;
double slice_thickness;
double via_distance;
double radial_steps;
double start_angle;
double end_angle;
double rib_downsampling;
double rib_spacing;
double z_vert_limit;
double rib_tolerance;
double max_rib_length;

    bool state;
    std::string name;

    
}groups;



//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double workspace;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double height_step;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double max_height;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double mid_height;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double offset;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double slice_thickness;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double via_distance;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double radial_steps;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double start_angle;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double end_angle;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double rib_downsampling;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double rib_spacing;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double z_vert_limit;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double rib_tolerance;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double max_rib_length;
//#line 218 "/opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("TrajectoryConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const TrajectoryConfig &__max__ = __getMax__();
      const TrajectoryConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const TrajectoryConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const TrajectoryConfig &__getDefault__();
    static const TrajectoryConfig &__getMax__();
    static const TrajectoryConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const TrajectoryConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void TrajectoryConfig::ParamDescription<std::string>::clamp(TrajectoryConfig &config, const TrajectoryConfig &max, const TrajectoryConfig &min) const
  {
    return;
  }

  class TrajectoryConfigStatics
  {
    friend class TrajectoryConfig;
    
    TrajectoryConfigStatics()
    {
TrajectoryConfig::GroupDescription<TrajectoryConfig::DEFAULT, TrajectoryConfig> Default("Default", "", 0, 0, true, &TrajectoryConfig::groups);
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.workspace = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.workspace = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.workspace = 0.5;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("workspace", "double", 0, "Workspace Width", "", &TrajectoryConfig::workspace)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("workspace", "double", 0, "Workspace Width", "", &TrajectoryConfig::workspace)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.height_step = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.height_step = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.height_step = 0.25;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("height_step", "double", 0, "Height step", "", &TrajectoryConfig::height_step)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("height_step", "double", 0, "Height step", "", &TrajectoryConfig::height_step)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.max_height = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.max_height = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.max_height = 1.3;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("max_height", "double", 0, "Maximum reachable height", "", &TrajectoryConfig::max_height)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("max_height", "double", 0, "Maximum reachable height", "", &TrajectoryConfig::max_height)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.mid_height = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.mid_height = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.mid_height = 0.75;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("mid_height", "double", 0, "Midpoint to calculate ribs on", "", &TrajectoryConfig::mid_height)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("mid_height", "double", 0, "Midpoint to calculate ribs on", "", &TrajectoryConfig::mid_height)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.offset = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.offset = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.offset = 0.3;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("offset", "double", 0, "distance from wall to end effector", "", &TrajectoryConfig::offset)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("offset", "double", 0, "distance from wall to end effector", "", &TrajectoryConfig::offset)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.slice_thickness = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.slice_thickness = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.slice_thickness = 0.03;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("slice_thickness", "double", 0, "Thickness ofribs", "", &TrajectoryConfig::slice_thickness)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("slice_thickness", "double", 0, "Thickness ofribs", "", &TrajectoryConfig::slice_thickness)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.via_distance = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.via_distance = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.via_distance = 0.2;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("via_distance", "double", 0, "Distance between via points", "", &TrajectoryConfig::via_distance)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("via_distance", "double", 0, "Distance between via points", "", &TrajectoryConfig::via_distance)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.radial_steps = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.radial_steps = 100.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.radial_steps = 50.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("radial_steps", "double", 0, "number of ribs to make", "", &TrajectoryConfig::radial_steps)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("radial_steps", "double", 0, "number of ribs to make", "", &TrajectoryConfig::radial_steps)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.start_angle = -10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.start_angle = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.start_angle = -1.4;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("start_angle", "double", 0, "starting angle for ribs", "", &TrajectoryConfig::start_angle)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("start_angle", "double", 0, "starting angle for ribs", "", &TrajectoryConfig::start_angle)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.end_angle = -10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.end_angle = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.end_angle = 1.3;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("end_angle", "double", 0, "ending angle for ribs", "", &TrajectoryConfig::end_angle)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("end_angle", "double", 0, "ending angle for ribs", "", &TrajectoryConfig::end_angle)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.rib_downsampling = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.rib_downsampling = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.rib_downsampling = 0.05;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_downsampling", "double", 0, "distance between rib points", "", &TrajectoryConfig::rib_downsampling)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_downsampling", "double", 0, "distance between rib points", "", &TrajectoryConfig::rib_downsampling)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.rib_spacing = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.rib_spacing = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.rib_spacing = 0.05;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_spacing", "double", 0, "distance between ribs", "", &TrajectoryConfig::rib_spacing)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_spacing", "double", 0, "distance between ribs", "", &TrajectoryConfig::rib_spacing)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.z_vert_limit = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.z_vert_limit = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.z_vert_limit = 1.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("z_vert_limit", "double", 0, "limit for z value to only use Z for distance", "", &TrajectoryConfig::z_vert_limit)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("z_vert_limit", "double", 0, "limit for z value to only use Z for distance", "", &TrajectoryConfig::z_vert_limit)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.rib_tolerance = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.rib_tolerance = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.rib_tolerance = 0.05;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_tolerance", "double", 0, "maximim distance a rib point can be from target point", "", &TrajectoryConfig::rib_tolerance)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("rib_tolerance", "double", 0, "maximim distance a rib point can be from target point", "", &TrajectoryConfig::rib_tolerance)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.max_rib_length = 0.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.max_rib_length = 10.0;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.max_rib_length = 2.26;
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("max_rib_length", "double", 0, "maximum distance along rib to calculate trajectory points", "", &TrajectoryConfig::max_rib_length)));
//#line 280 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(TrajectoryConfig::AbstractParamDescriptionConstPtr(new TrajectoryConfig::ParamDescription<double>("max_rib_length", "double", 0, "maximum distance along rib to calculate trajectory points", "", &TrajectoryConfig::max_rib_length)));
//#line 235 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 235 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(TrajectoryConfig::AbstractGroupDescriptionConstPtr(new TrajectoryConfig::GroupDescription<TrajectoryConfig::DEFAULT, TrajectoryConfig>(Default)));
//#line 353 "/opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<TrajectoryConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<TrajectoryConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<TrajectoryConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    TrajectoryConfig __max__;
    TrajectoryConfig __min__;
    TrajectoryConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const TrajectoryConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static TrajectoryConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &TrajectoryConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const TrajectoryConfig &TrajectoryConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const TrajectoryConfig &TrajectoryConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const TrajectoryConfig &TrajectoryConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<TrajectoryConfig::AbstractParamDescriptionConstPtr> &TrajectoryConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<TrajectoryConfig::AbstractGroupDescriptionConstPtr> &TrajectoryConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const TrajectoryConfigStatics *TrajectoryConfig::__get_statics__()
  {
    const static TrajectoryConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = TrajectoryConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __TRAJECTORYRECONFIGURATOR_H__
