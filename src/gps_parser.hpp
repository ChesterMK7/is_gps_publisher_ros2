// Inertial Sense GPS & IMU data parser header file for Scout 2.0 Autonomous Navigation Project
// Version 1.0.6 (2025-07-01)

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
        void setData(double d[7], int i[7], char c[2], std::string u); 
        void printData();
        sensor_msgs::msg::Imu IMUoutputROS();
        sensor_msgs::msg::NavSatFix GPSoutputROS();
        int dataType() {return ia[6];}
    private:
        double da[7];
        // For DGGA: ia[4] = utc_hr, ia[5] = utc_min;
        int ia[7];
        char ca[2];
        std::string sv;
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
    if (ia[2] == 1) status = 0;
    else if (ia[2] == 2) status = 1;
    else if (ia[2] == 4 ||  ia[2] == 5) status = 2;
    else status = -1;
    statusGPS.status = status;
    statusGPS.service = 1;
    output.status = statusGPS;
    // latitude [degrees]. Positive is north of equator; negative is south.
    double lat = ia[0] + (da[1]/60);
    if (ca[0] == 'S') lat = lat * -1;
    output.latitude = lat;
    // longitude [degrees]. Positive is east of prime meridian; negative is west.
    double lon = ia[1] + (da[2]/60);
    if (ca[1] == 'W') lon = lon * -1;
    output.longitude = lon;
    // altitude [m]. Positive is above the WGS 84 ellipsoid
    output.altitude = da[4];
    double position_covariance[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double hdop = da[3];
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
        geometry_msgs::msg::Vector3 angular_velocity;
        geometry_msgs::msg::Vector3 linear_acceleration;
        angular_velocity.x = da[1];
        angular_velocity.y = da[2];
        angular_velocity.z = da[3];
        linear_acceleration.x = da[4];
        linear_acceleration.y = da[5];
        linear_acceleration.z = da[6];
        output.angular_velocity = angular_velocity;
        output.linear_acceleration = linear_acceleration;
        for (int i = 0; i < 9; i++) {
        	output.orientation_covariance[i] = orientation_covariance[i];
        	output.angular_velocity_covariance[i] = angular_velocity_covariance[i];
        	output.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
        }
        return output;
}

void msgdata::setData(double d[7], int i[7], char c[2], std::string u) {
    for (int i = 0; i < 7; i++) da[i] = d[i];
    for (int j = 0; j < 7; j++) ia[j] = i[j];
    ca[0] = c[0]; ca[1] = c[1]; sv = u;
}

void msgdata::printData() {
    if (ia[6] == 1) {
        std::string fq[9] = {"invalid","GPS fix","DGPS fix","PPS fix","RTK Fix","RTK Float","estimated","Manual input mode","Simulation mode"};
       std::cout << "GPS Data\n";
       std::cout << "UTC Time: " << sv[0] << sv[1] << ":";
       std::cout << sv[2] << sv[3] << ":";
        for (int i = 4; i < sv.size(); i++) {
           std::cout << sv[i];
        }
       std::cout << "\nLatitude: " << ia[0] << " Degrees, " << da[1] << " Minutes " << ca[0];
       std::cout << "\nLongitude: " << ia[1] << " Degrees, " << da[2] << " Minutes " << ca[1];
       std::cout << "\nAltitude: " << da[4] << " Meters";
       std::cout << "\nHorizontal dilution of precision: " << da[3] << " Meters";
       std::cout <<"\nUndulation: " << da[5] << " Meters";
       std::cout << "\nFix Quality: " << fq[ia[2]] << "\nSatilites Used: " << ia[3] << "\n";
        if (da[6] != 0) {std::cout << "Seconds since last update: " << da[6] << "\n";}
    }
    else if (ia[6] == 2) {
       std::cout << "IMU Data\n";
       std::cout << "Time: " << da[0] << "s\n";
       std::cout << "Angular Rate Gyro (rad/sec)\nX: " << da[1] << " Y: " << da[2];
       std::cout << " Z: " << da[3] << "\nLinear Accel(m/s/s)\nX: " << da[4];
       std::cout << " Y: " << da[5] << " Z: " << da[6] << "\n";
    }
    else std::cout << "Indvalid Data Type " << ia[6] << "\n";
}

msgdata coordParser (std::string inp) {
    std::string temp, temp2 = ""; 
    int min, form, field, dec, ofs;
    min = form = field = dec = ofs = 0;
    int valI[7] = {0,0,0,0,0,0};
    double valD[7] = {0,0,0,0,0,0,0};
    char valC[2] = {' ',' '};
    std::string valS = "";
    msgdata outp;
    for (int i = 0; i < inp.size(); i++) {
        if (inp[i] == ',' || i + 1 == inp.size() || inp[i] == '*') {
            temp = "";
            temp2 = "";
            for (int k = min; k < i; k++) {
                temp = temp + inp[k];
            }
            //cout << temp << "\n";
            if (min == 0) {
                if (temp.length() == 6) {
                    temp2 = temp2 + temp[3] + temp[4] + temp[5];
                    if (temp2.compare("GGA") == 0) form = 1;
                    else form = -1;
                }
                else if (temp.compare("$PIMU") == 0) form = 2;
                else form = -1;
                temp2 = "";
                valI[6] = form;
            }
            else {
                //cout << form << " | " << field << "\n";
                //&& field > 0
                if (form == 2 && field <= 8) {
                    valD[field-1] = stod(temp);
                }
                if (form == 1 && field <= 13) {
                    if (field == 3) valC[0] = temp[0];
                    else if (field == 5) valC[1] = temp[0];
                    else if (field == 6) valI[2] = stoi(temp);
                    else if (field == 7) valI[3] = stoi(temp);
                    else if (field == 8) valD[3] = stod(temp);
                    else if (field == 9) valD[4] = stod(temp);
                    else if (field == 11) valD[5] = stod(temp);
                    else if (field == 13) {
                        if (temp != "") valD[6] = stod(temp);
                    }
                    else if (field == 1 || field == 2 || field == 4) {
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
                        if (field == 2) {valD[1] = stod(temp2);}
                        else if (field == 4) valD[2] = stod(temp2);
                        else valD[0] = stod(temp2);
                        temp2 = "";
                        ofs = dec - 3;
                        if (field == 2 || field == 4) {
                            if (ofs >= 0) {
                                for (int n = 0; n <= ofs; n++) {
                                    temp2 += temp[n];
                                }
                            }
                            else temp2 = "0";
                            if (field == 2) {valI[0] = stoi(temp2);}
                            else valI[1] = stoi(temp2);
                        }
                        else {
                            valS = temp;
                            if (ofs >= 1) {
                                temp2 += temp[ofs-1];
                                temp2 += temp[ofs];
                                valI[5] = stoi(temp2);
                                temp2 = "";
                                if (ofs == 2) {
                                    temp2 += temp[ofs-2];
                                }
                                else if (ofs == 3) {
                                    temp2 += temp[ofs-3];
                                    temp2 += temp[ofs-2];
                                }
                                if (temp2 != "") valI[4] = stoi(temp2);
                                else valI[4] = 0;
                            }
                            else {
                                valI[4] = 0;
                                if (ofs == 0) {
                                    temp2 += temp[0];
                                    valI[5] = stoi(temp2);
                                }
                                else valI[5] = 0;
                            }
                        }
                    }
                }
            } 
        min = i+1;
        field++;
        }
    }
    outp.setData(valD,valI,valC,valS);
    return outp;
}

builtin_interfaces::msg::Time getTime(double secs) {
    int s = secs;
    uint32_t ns = (secs - s)*1000000000;
    builtin_interfaces::msg::Time output;
    output.sec = s;
    output.nanosec = ns;
    //std::cout << std::to_string(secs) << "s\n" << s << "s\n" << ns << "ns\n";
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
