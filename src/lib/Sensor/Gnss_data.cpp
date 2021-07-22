#include "Sensor/Gnss_data.h"

namespace Sensor{

    // 静态变量全局初始化
    GnssDataProcess* GnssDataProcess::gnss_data_process_ptr = nullptr;  
    std::mutex GnssDataProcess::m_gnss_data_process;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    GnssDataProcess* GnssDataProcess::GetInstance()
    {
        if(gnss_data_process_ptr == nullptr)
        {
            std::unique_lock<std::mutex> lock(m_gnss_data_process);
            gnss_data_process_ptr = new GnssDataProcess{};    // 实例化 
        }
        
        return gnss_data_process_ptr;  
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void GnssDataProcess::InitOriginPosition(Eigen::Vector3d const& llt)
    {
        geo_converter.Reset(llt[0], llt[1], llt[2]);
        enu_xyz = {0,0,0};
        origin_position_inited = true;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GnssDataProcess::UpdateXYZ(Eigen::Vector3d const& llt, Eigen::Vector3d &enu)
    {
        if (!origin_position_inited) 
        {
            std::cout<< "GeoConverter has not set origin position"<<std::endl;
            return false; 
        }

        geo_converter.Forward(llt[0], llt[1], llt[2], enu[0], enu[1], enu[2]);
        enu_xyz[0] = enu[0];
        enu_xyz[1] = enu[1];
        enu_xyz[2] = enu[2];

        return true; 

    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GnssDataProcess::UpdateXYZ(Eigen::Vector3d const& llt)
    {
        if (!origin_position_inited) 
        {
            std::cout<< "GeoConverter has not set origin position"<<std::endl;
            return false; 
        }

        geo_converter.Forward(llt[0], llt[1], llt[2], enu_xyz[0], enu_xyz[1], enu_xyz[2]);
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 获取ENU坐标  
    Eigen::Vector3d const& GnssDataProcess::GetEnuPosition() const
    {
        return enu_xyz;  
    }


} // namespace Sensor 
