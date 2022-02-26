#include <encoder_arduino_bridge/encoder_arduino_bridge.h>

// Constant definitions
const string EncoderArduinoBridge::NODE_NAME = "encoder_arduino_bridge";
const string EncoderArduinoBridge::PACKET_END = "\n";

const string EncoderArduinoBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string EncoderArduinoBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string EncoderArduinoBridge::START_COMMAND = "g" + PACKET_END;
const string EncoderArduinoBridge::STOP_COMMAND = "s" + PACKET_END;
const string EncoderArduinoBridge::ENCODER_MESSAGE_HEADER = "enc";
const string EncoderArduinoBridge::MESSAGE_DELIMITER = "\t";
const string EncoderArduinoBridge::INT64_SEGMENT_DELIMITER = "|";

// convert a string to a 64-bit integer
long long string_to_int64(string s) {
    stringstream ss(s);
    long long integer = 0;
    ss >> integer;
    return integer;
}

EncoderArduinoBridge::EncoderArduinoBridge(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // pull parameters from the launch file
    nh.param<string>("serial_port", serial_port, "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_00FEBA77-if00-port0");
    nh.param<int>("serial_baud", serial_baud, 115200);

    #ifdef USE_ENCODER1
    enc1_pub = nh.advertise<std_msgs::Int64>("/encoder1_raw", 100);
    #endif

    #ifdef USE_ENCODER2
    enc2_pub = nh.advertise<std_msgs::Int64>("/encoder2_raw", 100);
    #endif
}


//Wait for packet, and if can't then exit
void EncoderArduinoBridge::waitForPacket(const string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(15.0);

    //Attempt until timeout
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

int EncoderArduinoBridge::run()
{
    // attempt to open the serial port
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

    // wait for startup messages from the microcontroller
    waitForPacket(HELLO_MESSAGE);
    waitForPacket(READY_MESSAGE);

    // tell the microcontroller to start
    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(60);  // run loop at 60 Hz

    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        // if the serial buffer has data, parse it
        if (serial_ref.available())
        {
            serial_buffer = serial_ref.readline();

            // If the serial buffer starts with '-', the microcontroller is relaying a message for the user
            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse encoder segment
            if (serial_buffer.length() > ENCODER_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, ENCODER_MESSAGE_HEADER.size(), ENCODER_MESSAGE_HEADER) == 0) {
                parseEncoderMessage();
            }
        }
    }

    // tell the microcontroller to stop
    serial_ref.write(STOP_COMMAND);

    return 0;
}


//Decode Int64 message 
int64_t EncoderArduinoBridge::parseSegmentedInt64(string s) {
    size_t pos = s.find(INT64_SEGMENT_DELIMITER);
    if (pos == string::npos) {
        ROS_WARN("Couldn't find int 64 delimiting character in encoder message: '%s'", s.c_str());
        return 0;
    }


    //Parse the string and convert it into long
    long long part1 = string_to_int64(s.substr(0, pos));
    long long part2 = string_to_int64(s.substr(pos + 1));

    return (part1 << 32) | part2;
}


//Choose which encoder to use based on the message
void EncoderArduinoBridge::parseToken(string token) {
    switch (token.at(0)) {
        case 't': ROS_DEBUG("encoder arduino time: %s", token.substr(1).c_str()); break;


        //Read toeken data and then publish
        #ifdef USE_ENCODER1
        case 'a':
            enc1_msg.data = parseSegmentedInt64(token.substr(1));
            enc1_pub.publish(enc1_msg);
            break;
        #endif

        #ifdef USE_ENCODER2
        case 'b':
            enc2_msg.data = parseSegmentedInt64(token.substr(1));
            enc2_pub.publish(enc2_msg);
            break;
        #endif
        
        default:
            ROS_WARN("Invalid segment type! Segment: '%s', packet: '%s'", token.c_str(), serial_buffer.c_str());
            break;
    }
}

void EncoderArduinoBridge::parseEncoderMessage()
{
    // strip off header and the trailing newline character
    serial_buffer = serial_buffer.substr(ENCODER_MESSAGE_HEADER.size() + 1, serial_buffer.size()-1);

    // loop through the buffer until the end is reached
    // pos is the next MESSAGE_DELIMITER character
    size_t pos = 0;
    string token;
    while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
    {
        // extract the next segment of data (serial_buffer will be erased up to pos at the end)
        token = serial_buffer.substr(0, pos);
        if (token.size() == 0) {
            continue;
        }

        parseToken(token);

        // erase up to the end of the current token plus the delimiter character
        serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
    }

    // parse the rest of the serial buffer except for end character
    token = serial_buffer.substr(0, serial_buffer.length() - 1);
    parseToken(token);
}
