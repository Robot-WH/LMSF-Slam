// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

    double toc(std::string str)
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double time = elapsed_seconds.count() * 1000;
        std::cout<< str << "time: " << time <<std::endl;
        return time;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
