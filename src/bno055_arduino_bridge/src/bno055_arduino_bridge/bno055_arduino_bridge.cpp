#include <bno055_arduino_bridge/bno055_arduino_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string Bno055ArduinoBridge::IMU_FRAME_ID = "Imu";
const string Bno055ArduinoBridge::CHILD_FRAME_ID = "base_link";

const string Bno055ArduinoBridge::NODE_NAME = "bno055_arduino_bridge";
const string Bno055ArduinoBridge::PACKET_END = "\n";

const string Bno055ArduinoBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string Bno055ArduinoBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string Bno055ArduinoBridge::START_COMMAND = "g" + PACKET_END;
const string Bno055ArduinoBridge::STOP_COMMAND = "s" + PACKET_END;
const string Bno055ArduinoBridge::IMU_MESSAGE_HEADER = "imu";
const string Bno055ArduinoBridge::MESSAGE_DELIMITER = "\t";

const string Bno055ArduinoBridge::ENCODER_MESSAGE_HEADER = "e";

const size_t Bno055ArduinoBridge::MOTOR_COMMAND_MESSAGE_LEN = 17;

const float Bno055ArduinoBridge::JUMP_WARN_THRESHOLD = 0.5; // radians

long long string_to_int64(string s) {
    stringstream ss(s);
    long long integer = 0;
    ss >> integer;
    return integer;
}

Bno055ArduinoBridge::Bno055ArduinoBridge(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    nh.param<string>("serial_port", serial_port, "usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_00FEBA3D-if00-port0");
    nh.param<int>("serial_baud", serial_baud, 115200);

    imu_pub = nh.advertise<sensor_msgs::Imu>("/BNO055", 5);

    euler_roll = 0.0;
    euler_pitch = 0.0;
    euler_yaw = 0.0;

    prev_euler_roll = 0.0;
    prev_euler_pitch = 0.0;
    prev_euler_yaw = 0.0;

    system_status = 0;
    accel_status = 0;
    gyro_status = 0;
    mag_status = 0;
}


void Bno055ArduinoBridge::waitForPacket(const string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);

    while ((ros::Time::now() - begin) < timeout)
    {
        if (serial_ref.available()) {
            serial_buffer = serial_ref.readline();
            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.compare(packet) == 0) {
                ROS_INFO("Arduino sent %s!", packet.c_str());
                return;
            }
        }
    }

    throw Error("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
}

void Bno055ArduinoBridge::eulerToQuat(sensor_msgs::Imu &imu_msg, float roll, float pitch, float yaw)
{
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	imu_msg.orientation.w = cy * cr * cp + sy * sr * sp;
	imu_msg.orientation.x = cy * sr * cp - sy * cr * sp;
	imu_msg.orientation.y = cy * cr * sp + sy * sr * cp;
	imu_msg.orientation.z = sy * cr * cp - cy * sr * sp;
}

int Bno055ArduinoBridge::run()
{
    try
    {
        serial_ref.setPort(serial_port);
        serial_ref.setBaudrate(serial_baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_ref.setTimeout(timeout);
        serial_ref.open();
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }

    waitForPacket(HELLO_MESSAGE);
    waitForPacket(READY_MESSAGE);

    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(60);  // 60 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        clock_rate.sleep();

        if (serial_ref.available())
        {
            serial_buffer = serial_ref.readline();

            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse IMU segment
            if (serial_buffer.length() > IMU_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, IMU_MESSAGE_HEADER.size(), IMU_MESSAGE_HEADER) == 0) {
                parseImuMessage();
            }
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}

void Bno055ArduinoBridge::parseImuMessage()
{
    // strip off header and the trailing newline character
    serial_buffer = serial_buffer.substr(IMU_MESSAGE_HEADER.size() + 1, serial_buffer.size()-1);

    imu_msg.header.frame_id = IMU_FRAME_ID;
    imu_msg.header.stamp = ros::Time::now();

    size_t pos = 0;
    string token;
    while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
    {
        token = serial_buffer.substr(0, pos);
        if (token.size() == 0) {
            break;
        }

        switch (token.at(0)) {
            case 't': ROS_DEBUG("imu arduino time: %s", token.substr(1).c_str()); break;
            case 'e':
                switch (token.at(1)) {
                    case 'x': euler_roll = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': euler_pitch = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': euler_yaw = M_PI / 180.0 * STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            // case 'a':
            //
            //     break;
            case 'g':
                switch (token.at(1)) {
                    case 'x': imu_msg.angular_velocity.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.angular_velocity.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.angular_velocity.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            // case 'm':
            //
            //     break;
            case 'l':
                switch (token.at(1)) {
                    case 'x': imu_msg.linear_acceleration.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.linear_acceleration.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.linear_acceleration.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            case 'q':
                switch (token.at(1)) {
                    case 'w': imu_msg.orientation.w = STR_TO_FLOAT(token.substr(2)); break;
                    case 'x': imu_msg.orientation.x = STR_TO_FLOAT(token.substr(2)); break;
                    case 'y': imu_msg.orientation.y = STR_TO_FLOAT(token.substr(2)); break;
                    case 'z': imu_msg.orientation.z = STR_TO_FLOAT(token.substr(2)); break;
                }
                break;
            case 's':
                switch (token.at(1)) {
                    case 's': system_status = STR_TO_INT(token.substr(2)); break;
                    case 'g': accel_status = STR_TO_INT(token.substr(2)); break;
                    case 'a': gyro_status = STR_TO_INT(token.substr(2)); break;
                    case 'm': mag_status = STR_TO_INT(token.substr(2)); break;
                }
                break;
            default:
                ROS_WARN("Invalid segment type! Segment: '%s', packet: '%s'", token.c_str(), serial_buffer.c_str());
                break;
        }

        serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
    }

    if (system_status <= 1) {
        ROS_WARN("System status is %i! Sensor data may be invalid!!", system_error);
    }

    ROS_DEBUG("system status: %i", system_status);
    ROS_DEBUG("gyro status: %i", accel_status);
    ROS_DEBUG("accel status: %i", gyro_status);
    ROS_DEBUG("mag status: %i", mag_status);

    if (euler_roll - prev_euler_roll > JUMP_WARN_THRESHOLD) {
        ROS_WARN("bno055 roll jumped suddenly (amount: %f)", euler_roll - prev_euler_roll);
    }
    if (euler_pitch - prev_euler_pitch > JUMP_WARN_THRESHOLD) {
        ROS_WARN("bno055 pitch jumped suddenly (amount: %f)", euler_pitch - prev_euler_pitch);
    }
    if (euler_yaw - prev_euler_yaw > JUMP_WARN_THRESHOLD) {
        ROS_WARN("bno055 yaw jumped suddenly (amount: %f)", euler_yaw - prev_euler_yaw);
    }

    prev_euler_roll = euler_roll;
    prev_euler_pitch = euler_pitch;
    prev_euler_yaw = euler_yaw;

    eulerToQuat(imu_msg, euler_roll, euler_pitch, euler_yaw);

    imu_pub.publish(imu_msg);
}
