/*
 * Copyright(c)2018 ChangSha XingShen Technology Ltd.
 *
 * All rights reserved
 * Author:    wanghao
 *-------------------------------
 *Changes:
 *-------------------------------
 * v1.0 2018-08-06 :created by wanghao
 *
 */

#ifndef __common__ini__
#define __common__ini__

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/serialization/singleton.hpp>  //ubuntu
#include <exception>
#include <iostream>
#include <string>

/*
 *
 *    使用boost::property_tree 读取ini文件
 *
 *
 */
class IniConf {
 public:
  inline int openFile(const char *fileConf);

  inline int getValue(std::string Key, int Default, char Sep = '.');

  inline std::string getValue(std::string Key, std::string Default,
                              char Sep = '.');

  inline int putValue(std::string Key, double Default, char Sep = '.');
  inline int putValue(std::string Key, int Default, char Sep = '.');
  inline int putValue(std::string Key, std::string Default, char Sep = '.');

  inline int closeFile(const char *fileConf);

 private:
  boost::property_tree::ptree m_pt;
};

inline int IniConf::openFile(const char *fileConf) {
  if (NULL == fileConf || fileConf[0] == 0) {
    return 0;
  }
  try {
    boost::property_tree::read_ini(fileConf, m_pt);
  } catch (std::exception &e) {
    std::cout << "iniConf::openFile -- parse file failed :" << e.what()
              << std::endl;
    return 0;
  }
  return 1;
}

inline int IniConf::getValue(std::string Key, int Default, char Sep) {
  try {
    return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
  } catch (std::exception &e) {
    std::cout << "iniConf::getvalue -- get value feiled:" << e.what()
              << std::endl;
    return Default;
  }
}

inline std::string IniConf::getValue(std::string Key, std::string Default,
                                     char Sep) {
  try {
    return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
  } catch (std::exception &e) {
    std::cout << "iniConf::getvalue -- get value feiled:" << e.what()
              << std::endl;
    return Default;
  }
}

inline int IniConf::putValue(std::string Key, double Default, char Sep) {
  try {
    m_pt.put<double>(boost::property_tree::ptree::path_type(Key, Sep), Default);
    return 1;
  } catch (std::exception &e) {
    std::cout << "iniConf::putVaule -- put value failed" << e.what()
              << std::endl;
    return 0;
  }
}

inline int IniConf::putValue(std::string Key, int Default, char Sep) {
  try {
    m_pt.put<int>(boost::property_tree::ptree::path_type(Key, Sep), Default);
    return 1;
  } catch (std::exception &e) {
    std::cout << "iniConf::putVaule -- put value failed" << e.what()
              << std::endl;
    return 0;
  }
}

inline int IniConf::putValue(std::string Key, std::string Default, char Sep) {
  try {
    m_pt.put<std::string>(boost::property_tree::ptree::path_type(Key, Sep),
                          Default);
    return 1;
  } catch (std::exception &e) {
    std::cout << "iniConf::putVaule -- put value failed" << e.what()
              << std::endl;
    return 0;
  }
}

inline int IniConf::closeFile(const char *fileConf) {
  boost::property_tree::ini_parser::write_ini(fileConf, m_pt);
}

typedef boost::serialization::singleton<IniConf> GlobeIniConf;  // ubuntu
#define SINGLETON_INICONF GlobeIniConf::get_mutable_instance()  // ubuntu

#endif
