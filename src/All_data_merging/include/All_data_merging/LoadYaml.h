#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <exception>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <fstream>
#include <ros/package.h>
#include "All_data_merging/SerialData.h"
#include "All_data_merging/Name_Value.h"



class LoadYaml
{
public:
    // YAML 配置相关
    static YAML::Node yaml_config_;
    static bool config_loaded_;
    std::vector<std::string> device_names_;  // 存储所有设备名称
    std::vector<std::string> data_names_;  // 存储所有数据字段名称
    std::vector<std::string> data_types_;  // 存储所有数据字段类型
    std::vector<double> data_values_;       // 存储解析出来的数据值
    
    // 加载 YAML 配置文件
    static void loadYamlConfig();

    //加载数据名称
    void Load_name();
    
    // 使用 YAML 配置解析原始字节数据
    void parseFromBytes(const std::string& raw_payload);
    
    // 填充ROS消息
    void fill_msg(All_data_merging::SerialData& msg);
        
    void scan_header_footer();
};