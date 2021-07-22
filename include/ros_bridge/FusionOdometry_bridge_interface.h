#ifndef _FUSIONODOMETRY_BRIDGE_H_
#define _FUSIONODOMETRY_BRIDGE_H_

#include "utility.hpp"
#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"

using namespace Sensor; 

class FusionOdometryBridgeInterface  
{
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        // 参数服务
        ParamServer param_server_;  
        
    public:    

        FusionOdometryBridgeInterface()
        {
        }
        virtual ~FusionOdometryBridgeInterface()
        {   
        }

        // 处理线程 
        virtual void Process() = 0; 
};


#endif