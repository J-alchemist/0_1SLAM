// BOOST读写文件管理

#ifndef __TOOLS_FILE_MANAGER_H
#define __TOOLS_FILE_MANAGER_H

#include <string>
#include <iostream>
#include <fstream>


class FileManager {
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};


#endif
