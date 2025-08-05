#include "All_data_merging/LoadYaml.h"
#include <sstream>
#include <iomanip>


// 静态成员变量定义
YAML::Node LoadYaml::yaml_config_ = YAML::Node();
bool LoadYaml::config_loaded_ = false;

// 外部声明的全局变量
extern std::string head_str, tail_str;


void LoadYaml::loadYamlConfig() {
    if (config_loaded_) return;
    
    try {
        // 使用ROS包路径查找配置文件
        std::string package_path = ros::package::getPath("All_data_merging");
        std::string config_file = package_path + "/config/serial_frame_format.yaml";
        
        // 加载YAML配置
        yaml_config_ = YAML::LoadFile(config_file);
        config_loaded_ = true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load YAML config: %s", e.what());
        config_loaded_ = false;
    }
}

void LoadYaml::scan_header_footer()
{
    // 从 YAML 配置动态读取帧头尾标识符
    if (LoadYaml::config_loaded_) {
            YAML::Node frame_config = LoadYaml::yaml_config_["frame"];
            
            // 读取帧头
            if (frame_config["header"]) {
                auto header_bytes = frame_config["header"];
                head_str.clear();
                for (const auto& byte_val : header_bytes) {
                    head_str.append(1, static_cast<char>(byte_val.as<int>()));
                }
                ROS_INFO("Frame header loaded from YAML: %d bytes", (int)head_str.size());
            } 
            // 读取帧尾
            if (frame_config["footer"]) {
                auto footer_bytes = frame_config["footer"];
                tail_str.clear();
                for (const auto& byte_val : footer_bytes) {
                    tail_str.append(1, static_cast<char>(byte_val.as<int>()));
                }
                ROS_INFO("Frame footer loaded from YAML: %d bytes", (int)tail_str.size());
            } 
            
    } 
    else {
        // YAML 配置未加载，使用默认值
        head_str.append(2, 0x51);  // 帧头：[0x51, 0x51]
        tail_str.append(1, '\r');  // 帧尾：[0x0D, 0x0A]
        tail_str.append(1, '\n');
        ROS_WARN("YAML config not loaded, using default frame markers");
    }
}

void LoadYaml::Load_name()
{    
    // 从 YAML 配置中读取数据名称
    auto frame_config = yaml_config_["frame"];
    auto payload_fields = frame_config["payload_fields"];
    
    device_names_.clear();  // 清空之前的设备名称
    data_names_.clear();  // 清空之前的数据名称
    data_types_.clear();  // 清空之前的数据类型
    
    // 动态读取所有设备和字段名称
    for (auto devices = payload_fields.begin(); devices != payload_fields.end(); ++devices) {
        std::string device_name = devices->first.as<std::string>();
        device_names_.push_back(device_name);  // 添加设备名称到数组
        
        auto device_data = devices->second;
        
        // 检查设备节点的类型
        if (device_data.IsMap()) {
            // 如果是Map类型，遍历其子字段
            for (auto data = device_data.begin(); data != device_data.end(); ++data) {
                std::string data_name = data->first.as<std::string>();
                std::string data_type = data->second.as<std::string>();
                
                // 添加字段名称和类型信息
                data_names_.push_back(device_name + "_" + data_name);
                data_types_.push_back(data_type); 
            }
        } else {
            // 如果是简单类型，直接使用设备名称
            std::string data_type = device_data.as<std::string>();
            data_names_.push_back(device_name);
            data_types_.push_back(data_type);  
        }
    }      
    // 打印所有读取到的数据名称和类型
    ROS_INFO("Data fields loaded from YAML: ");
    for (size_t i = 0; i < data_names_.size(); ++i) {
        ROS_INFO("  Data[%zu]: %s (Type: %s)", i, data_names_[i].c_str(), data_types_[i].c_str());
    }       
}

void LoadYaml::parseFromBytes(const std::string& raw_payload) {
    size_t byte_offset = 0;  // 原始字节数据中的偏移量
    
    // 清空之前的数据值
    data_values_.clear();
    data_values_.reserve(data_names_.size());
    
    // 遍历所有数据字段进行解析
    for (size_t i = 0; i < data_names_.size(); ++i) {
        const std::string& field_name = data_names_[i];
        const std::string& field_type = data_types_[i];
        
        // 根据数据类型确定需要读取的字节数
        size_t byte_count = 0;
        if (field_type == "int32_t" || field_type == "uint32_t") {
            byte_count = 4;
        } else if (field_type == "uint64_t" || field_type == "int64_t") {
            byte_count = 8;
        } else {
            ROS_WARN("Unsupported data type: %s for field: %s", field_type.c_str(), field_name.c_str());
            continue;
        }
        
        // 检查是否有足够的数据
        if (byte_offset + byte_count > raw_payload.size()) {
            ROS_WARN("Not enough data for field: %s (need %zu bytes, available %zu)", 
                     field_name.c_str(), byte_count, raw_payload.size() - byte_offset);
            break;
        }
        
        // 直接从字节数据中读取数值
        double parsed_value = 0.0;
        const char* data_ptr = raw_payload.data() + byte_offset;
        
        if (field_type == "int32_t") {
            int32_t* ptr = (int32_t*)data_ptr;
            parsed_value = static_cast<double>(*ptr);
        } else if (field_type == "uint32_t") {
            uint32_t* ptr = (uint32_t*)data_ptr;
            parsed_value = static_cast<double>(*ptr);
        } else if (field_type == "uint64_t") {
            uint64_t* ptr = (uint64_t*)data_ptr;
            parsed_value = static_cast<double>(*ptr);
            //parsed_value = static_cast<uint64_t>(*ptr);
        } else if (field_type == "int64_t") {
            int64_t* ptr = (int64_t*)data_ptr;
            parsed_value = static_cast<double>(*ptr);
            //parsed_value = static_cast<int64_t>(*ptr);
        }
        
        // 存储解析出的数据值
        data_values_.push_back(parsed_value);
        
        // 移动偏移量
        byte_offset += byte_count;
    }
}

void LoadYaml::fill_msg(All_data_merging::SerialData& msg) {
    // 设置header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "serial_frame";
    
    // 填充命名数据字段
    msg.data_fields.clear();
    msg.data_fields.reserve(data_names_.size());
    for (size_t i = 0; i < data_names_.size() && i < data_values_.size(); ++i) {
        All_data_merging::Name_Value named_value;
        named_value.name = data_names_[i];
        named_value.value = data_values_[i];
        msg.data_fields.push_back(named_value);
    }
    
    ROS_DEBUG("Filled message with %zu fields and %zu named data fields", data_names_.size(), msg.data_fields.size());
}
