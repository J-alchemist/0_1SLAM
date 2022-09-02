// BOOST文件读写的方法
#include "tools/file_manager.h"
#include <boost/filesystem.hpp>
#include <iostream>

/****** 创建文件 ****/
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {

    ofs.close();        // 防止别处在占用
    boost::filesystem::remove(file_path.c_str()); // 删除文件

    ofs.open(file_path.c_str(), std::ios::out); // 打开文件,没有就创建
    if (!ofs) {
        std::cout << "Can't Create this [file] in path: " << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}

/****** 创建目录 ****/
// 重载1
bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
    // delete
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path);  // 删除文件夹
    }
    // create
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "Can't Create this [directory] in path: " << directory_path << std::endl << std::endl;
        return false;
    }

    std::cout << use_for << "==> Save in path: " << directory_path << std::endl << std::endl;
    return true;
}
// 重载2
bool FileManager::CreateDirectory(std::string directory_path) {

    // delete
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path);
    }

    // create
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "Can't Create this [directory] in path: " << directory_path << std::endl << std::endl;
        return false;
    }

    return true;
}

