// Inertial Sense GPS & IMU data parser header file for Scout 2.0 Autonomous Navigation Project
// Version 1.2.0 (2026-07-02)

#define GPS_FRAME "gps_link" // GPS frame name
#define IMU_FRAME "imu_link" // IMU frame name
#define PORT 25565 // Port used
#define MAXLINE 256 
// Modified UDP server code that runs GPS parser
// Server side implementation of UDP client-server model 
#include <arpa/inet.h> 
#include <bits/stdc++.h> 
#include <chrono>
#include <functional>
#include <memory>
#include <sys/types.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <iostream>
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
using namespace std::chrono_literals;

class msgdata {
    public:
        void setGPSData(int lat_deg, double lat_min, char lat_dir, int lon_deg, double lon_min, char lon_dir, double h, double alt, int fix) {
            hdop = h; fix_qual = fix; altitude = alt;
            lat = lat_deg + (lat_min/60);
            lat = lat_deg + (lat_min/60);
            if (lat_dir == 'S') lat = lat * -1;
            lon = lon_deg + (lon_min/60);
            if (lon_dir == 'W') lon = lon * -1;
            msg_type = 1;
            std::cout << lat << " | " << lon << "\n";
        }
        void setIMUData(double ang_vel[3], double lin_accel[3]) {
            for (int i = 0; i < 3; i++) {
                angular_velocity[i] = ang_vel[i];
                linear_acceleration[i] = lin_accel[i];
            }
            msg_type = 2;
        }
        sensor_msgs::msg::Imu IMUoutputROS();
        sensor_msgs::msg::NavSatFix GPSoutputROS();
        int dataType() {return msg_type;}
    private:
        // Shared variable
        int msg_type = 0;
        // IMU Variables
        double angular_velocity[3];
        double linear_acceleration[3];
        // GGA Variables
        int fix_qual;
        double lat, lon, hdop, altitude;
};

// Driver code 
struct sockaddr_in setup(int sockfd) { 
    struct sockaddr_in servaddr, cliaddr; 
    if ( sockfd < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT);   
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    return cliaddr;
}

sensor_msgs::msg::NavSatFix msgdata::GPSoutputROS() {
    sensor_msgs::msg::NavSatFix output;
    sensor_msgs::msg::NavSatStatus statusGPS;
    int8_t status;
    switch(fix_qual) {
        case 1:
            status = 0;
            break;
        case 2:
            status = 1;
            break;
        case 4: case 5:
            status = 2;
            break;
        default:
            status = -1;
    }
    statusGPS.status = status;
    statusGPS.service = 1;
    output.status = statusGPS;
    output.latitude = lat;
    output.longitude = lon;
    output.altitude = altitude;
    double position_covariance[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    position_covariance[0] = hdop*hdop; // hDop^2
    position_covariance[4] = position_covariance[0];
    position_covariance[8] = 4 * position_covariance[0]; // (2*hDop)^2
    for (int i = 0; i < 9; i++) {
        output.position_covariance[i] = position_covariance[i];
    }
    output.position_covariance_type = 1;
    return output;
}

sensor_msgs::msg::Imu msgdata::IMUoutputROS() {
        sensor_msgs::msg::Imu output;
        double orientation_covariance[9] = {-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        double angular_velocity_covariance[9] = {0.05,0.0,0.0,0.0,0.05,0.0,0.0,0.0,0.05};
        double linear_acceleration_covariance[9] = {0.05,0.0,0.0,0.0,0.05,0.0,0.0,0.0,0.05};
        geometry_msgs::msg::Vector3 ang_vel;
        geometry_msgs::msg::Vector3 lin_accel;
        ang_vel.x = angular_velocity[0];
        ang_vel.y = angular_velocity[1];
        ang_vel.z = angular_velocity[2];
        lin_accel.x = linear_acceleration[0];
        lin_accel.y = linear_acceleration[1];
        lin_accel.z = linear_acceleration[2];
        output.angular_velocity = ang_vel;
        output.linear_acceleration = lin_accel;
        for (int i = 0; i < 9; i++) {
        	output.orientation_covariance[i] = orientation_covariance[i];
        	output.angular_velocity_covariance[i] = angular_velocity_covariance[i];
        	output.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
        }
        return output;
}

msgdata msgParser (std::string inp) {
    std::string temp, temp2 = ""; 
    int min, form, field, dec, ofs;
    min = form = field = dec = ofs = 0;
    int lat_deg, lon_deg, fix = 0;
    double lat_min, lon_min, hdop, alt = 0;
    double ang_vel[3], lin_accel[3] = {0,0,0};
    char lat_dir, lon_dir = ' ';
    msgdata outp;
    for (int i = 0; i < inp.size(); i++) {
        if (inp[i] == ',' || i + 1 == inp.size() || inp[i] == '*') {
            temp = "";
            temp2 = "";
            for (int k = min; k < i; k++) {
                temp = temp + inp[k];
            }
            if (min == 0) {
                if (temp.length() == 6) {
                    temp2 = temp2 + temp[3] + temp[4] + temp[5];
                    if (temp2.compare("GGA") == 0) form = 1;
                    else form = -1;
                }
                else if (temp.compare("$PIMU") == 0) form = 2;
                else form = -1;
                temp2 = "";
            }
            else {
                if (form == 2 && field <= 8) {
                    switch(field) {
                        case 2: case 3: case 4:
                            ang_vel[field-2] = stod(temp);
                            break;
                        case 5: case 6: case 7:
                            lin_accel[field-5] = stod(temp);
                            break;
                        default:
                            break;
                    }
                }
                if (form == 1 && field <= 13) {
                    switch(field) {
                        case 3:
                            lat_dir = temp[0];
                            break;
                        case 5:
                            lon_dir = temp[0];
                            break;
                        case 6:
                            fix = stoi(temp);
                            break;
                        case 8:
                            hdop = stod(temp);
                            break;
                        case 9:
                            alt = stod(temp);
                            break;
                        case 2: case 4:
                            for (int l = 0; l < temp.size(); l++) {
                                if (temp[l] == '.') {
                                    dec = l;
                                    l = temp.size();
                                }
                            }
                            temp2 = "";
                            if (dec >= 2) ofs = 2;
                            else ofs = dec;
                            for (int m = dec-ofs; m < temp.size(); m++) {
                                temp2 = temp2 + temp[m];
                            }
                            if (field == 2) lat_min = stod(temp2);
                            else lon_min = stod(temp2);
                            temp2 = "";
                            ofs = dec - 3;
                            if (ofs >= 0) {
                                for (int n = 0; n <= ofs; n++) {
                                    temp2 += temp[n];
                                }
                            }
                            else temp2 = "0";
                            if (field == 2) lat_deg = stoi(temp2);
                            else lon_deg = stoi(temp2);
                            break;
                        default:
                            break;
                    }
                }
            } 
        min = i+1;
        field++;
        }
    }
    if (form == 1) outp.setGPSData(lat_deg,lat_min,lat_dir,lon_deg,lon_min,lon_dir,hdop,alt,fix);
    else if (form == 2) outp.setIMUData(ang_vel,lin_accel);
    return outp;
}

builtin_interfaces::msg::Time getTime(double secs) {
    int s = secs;
    uint32_t ns = (secs - s)*1000000000;
    builtin_interfaces::msg::Time output;
    output.sec = s;
    output.nanosec = ns;
    return output;
}

std_msgs::msg::Header makeHeader(rclcpp::Time t) {
    builtin_interfaces::msg::Time rTime = getTime(t.seconds());
    std_msgs::msg::Header output;
    output.stamp = rTime;
    output.frame_id = "";
    return output;
}

/*
$xxGGA, UTC Time (HHMMSS.SSS), Latitude (DDMM.MMMMM), N/S, Longitude (DDDMM.MMMMM), E/W, Fix Quality, # Satilites, 
    hDop, M, Undulation, M, Secs Since Last Update,,*xx
$PIMU, Seconds Since Power On, Gyro X, Gyro Y, Gyro Z, Linear Accel X, Lin Accel Y, Lin Accel Z*xx
*/
