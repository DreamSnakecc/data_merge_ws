
#include "All_data_merging/serial_debug.h"
#include "All_data_merging/LoadYaml.h"

using namespace std;

//YAML配置加载
LoadYaml load_yaml;

string buffer;
string head_str, tail_str;

// 外部声明（在 serial_parser_pub_node.cpp 中定义）
extern Serial_Debug serial_debug;
extern ros::Publisher serial_data_pub;

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void SerialData_parser() {
    // 初始化 YAML 配置
    load_yaml.loadYamlConfig();
    
    // 设置帧头和帧尾
    load_yaml.scan_header_footer();
    load_yaml.Load_name();  

    while (ros::ok()) {
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait(lck, []{return ready;});
        ready = false;
        
        // 在这里执行耗时的数据处理操作
        static int t=0,tok=0;
        buffer += serial_debug.SerialRead();
        if(buffer.size() > 500)
        {
            buffer.clear();
        }
        int st = 0,en = 0;
        st = buffer.find(head_str);//找帧头
        if (st != std::string::npos)
        {
            en = buffer.find(tail_str,st);//找帧尾
            size_t payload_size = LoadYaml::yaml_config_["length"].as<size_t>() - 4;
            if (en != std::string::npos && en-st == payload_size + 2)
            {
                string valid_payload = buffer.substr(st+2, payload_size);  // 只提取载荷部分
                buffer.erase(buffer.begin(),buffer.begin()+en+tail_str.size());
                
                // 使用已经初始化好的 YAML 解析器处理原始字节数据
                load_yaml.parseFromBytes(valid_payload);
                
                // 发布消息
                All_data_merging::SerialData info_msg;
                load_yaml.fill_msg(info_msg);
                serial_data_pub.publish(info_msg);

                tok++;
            }
        }
        t++;
        if(t >= 200)
        {
            static ros::Time last_time, now_time;
            now_time = ros::Time::now();
            // double d = (double)((now_time.toSec() - last_time.toSec())*1e9 + (now_time.toNSec() - last_time.toNSec()))*1e-9;
            double d = (double)(now_time.toSec() - last_time.toSec());
            // ROS_INFO_STREAM("d:"<<d<<"s, rate:"<<tok/d<<"%");
            last_time = ros::Time::now();
            tok = 0;
            t = 0;
        }
    }
}

void SerialDataCallback(const ros::TimerEvent&)
{
    {
        std::lock_guard<std::mutex> lck(mtx);
        ready = true;
    }
    cv.notify_one();
    return ;
}

/*! 
 * \brief 初始化串口
 * 
 * \param port Jetson nano串口的端口
 */
void Serial_Debug::SerialInit(std::string port)
{
     //打开串口，波特率115200，串口为port
	try 
	{
		ros_ser_.setPort(port);             //串口端口设置
		ros_ser_.setBaudrate(115200);       //串口波特率设置
		serial::Timeout to = serial::Timeout::simpleTimeout(10);        //串口延时设置
		ros_ser_.setTimeout(to);
        ros_ser_.setStopbits(serial::stopbits_one);                     //一个停止位
        
		ros_ser_.open();        //打开串口
	} 
	catch (serial::IOException& e) 
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return ;
	}
	
	//输出提示消息
	if(ros_ser_.isOpen()) 
	{
         ROS_INFO_STREAM("Serial Port opened");
         ros_ser_.flushInput();
    } 
	else 
	{
        return;
    }
}

/*!
 * \brief 关闭串口
 */
void Serial_Debug::CloseSerial()
{
    ros_ser_.close();
    return;
}


/*!
 * \brief 读取串口数据
 * 
 * \return 串口缓冲区数据
 */
std::string Serial_Debug::SerialRead()
{
    std::string data_str;
    size_t p = ros_ser_.available();

    //串口数据读取
    if (p != 0)
    {
        data_str = ros_ser_.read(p);
        ros_ser_.flushInput();
    } 
    return data_str;
}

size_t Serial_Debug::SerialWrite(const std::string str)
{
    size_t n;
    n = ros_ser_.write(str);
    ROS_INFO("Tx: %s",(str).c_str());
    //ros_ser_.flushOutput();
    return n ;
}

void Serial_Debug::float2hex(float f, char h[4])
{
    char *ph = (char*)&f;
    for(int i=0; i<4; i++)
    {
        h[i] = ph[i];
    }
}
void Serial_Debug::double2hex(double d, char h[8])
{
    char *ph = (char*)&d;
    for(int i=0; i<8; i++)
    {
        h[i] = ph[i];
    }
}

