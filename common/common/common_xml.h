/*
 * Copyright(c)2018 ChangSha XingShen Technology Ltd.
 *
 * All rights reserved
 * Author:    wanghao
 *-------------------------------
 *Changes:
 *-------------------------------
 * v1.0 2018-08-15 :created by wanghao
 *
 */
#ifndef __common__xml__
#define __common__xml__

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/serialization/singleton.hpp>  //ubuntu
#include <exception>
#include <iostream>
#include <string>

/*
 *
 *    boost::property_tree
 *
 *
 */
class XmlConf {
 public:
  inline int openFile(const char *fileConf);

  inline int getValue(std::string Key, int Default, char Sep = '.');

  inline std::string getValue(std::string Key, std::string Default,
                              char Sep = '.');

 private:
  boost::property_tree::ptree m_pt;
};

inline int XmlConf::openFile(const char *fileConf) {
  if (NULL == fileConf || fileConf[0] == 0) {
    return 0;
  }
  try {
    boost::property_tree::read_xml(fileConf, m_pt);
  } catch (std::exception &e) {
    std::cout << "XmlConf::openFile -- parse file failed :" << e.what()
              << std::endl;
    return 0;
  }
  return 1;
}

typedef boost::serialization::singleton<XmlConf> GlobeXmlConf;  // ubuntu
#define SINGLETON_XMLCONF GlobeXmlConf::get_mutable_instance()  // ubuntu

#endif
