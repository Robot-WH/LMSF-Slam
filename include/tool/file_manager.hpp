
#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path) {
      ofs.close();
      boost::filesystem::remove(file_path.c_str());

      ofs.open(file_path.c_str(), std::ios::out);
      if (!ofs) {
          // LOG(WARNING) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
          std::cout<<" 无法生成文件: "<< file_path << std::endl;
          return false;
      }

      return true;
    }

    static bool InitDirectory(std::string directory_path, std::string use_for) {
      if (boost::filesystem::is_directory(directory_path)) {
          boost::filesystem::remove_all(directory_path);
      }

      return CreateDirectory(directory_path, use_for);
    }

    static bool CreateDirectory(std::string directory_path, std::string use_for) {
      if (!boost::filesystem::is_directory(directory_path)) {
          boost::filesystem::create_directory(directory_path);
      }

      if (!boost::filesystem::is_directory(directory_path)) {
          //LOG(WARNING) << "CANNOT create directory " << std::endl << directory_path << std::endl << std::endl;
          std::cout<<" CANNOT create directory "<<std::endl;
          return false;
      }

      std::cout << use_for << " output path:" << std::endl << directory_path << std::endl << std::endl;
      return true;
    }

    static bool CreateDirectory(std::string directory_path) {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }

        if (!boost::filesystem::is_directory(directory_path)) {
            // LOG(WARNING) << "CANNOT create directory " << std::endl << directory_path << std::endl << std::endl;
            std::cout<<" CANNOT create directory "<<std::endl;
            return false;
        }

        return true;
    }
};


#endif
