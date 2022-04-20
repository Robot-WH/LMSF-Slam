/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-30 16:31:48
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <map>
#include <cassert>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "color.hpp"

/**
 * @brief: 参数读取器     
 * @brief 读取 参数文件 config_file 中的参数   
 */
class ParametersReader 
{
    public:
        ParametersReader(std::string const& config_file)
        {
            openFile(config_file); 
        }
        // 读取参数   
        template <typename T>
        bool ReadParam(char* name, T &ans)
        {
            if (!fsSettings_.isOpened())
            {
                std::cerr << "ERROR: Wrong path to settings" << std::endl;
                return false;
            }
            ans = static_cast<T>(fsSettings_[name]);
            std::cout<<name<<": "<<ans<<std::endl;   
            return true;
        }

        bool IsOpened() const
        {
            if (!fsSettings_.isOpened())
            {
                std::cerr << "ERROR: Wrong path to settings" << std::endl;
                return false;
            }
            return true;  
        }
    
    private:

        bool openFile(std::string const& config_file) 
        {
            FILE *fh = fopen(config_file.c_str(),"r");
            if (fh == NULL)
            {
                std::cout << common::YELLOW << "config_file dosen't exist; wrong config_file path" 
                << common::RESET << std::endl;
                return  false;
            }
            fclose(fh);
            fsSettings_ = cv::FileStorage(config_file, cv::FileStorage::READ);
            if (!fsSettings_.isOpened())
            {
                std::cerr << "ERROR: Wrong path to settings" << std::endl;
                return false;
            }
            return true;  
        }

    private:
        cv::FileStorage fsSettings_;   
}; // class ParametersReader


