#ifndef NCURSES_FUNCTIONS_H
#define NCURSES_FUNCTIONS_H

#include <vector>
#include <ncurses.h>
#include <panel.h>
#include <string.h>
#include <cmath>

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "brov2_interfaces/msg/barometer.hpp"
#include "brov2_interfaces/msg/dvl.hpp"
#include "brov2_interfaces/msg/dvl_beam.hpp"
#include "brov2_interfaces/msg/dvl_odom.hpp"
#include "bluerov_interfaces/msg/reference.hpp"


class NcursesFunctions
{
public:
    NcursesFunctions();
    ~NcursesFunctions();

    std::vector<WINDOW*> InitWindows();
    void InitColors();
    void DrawBox(WINDOW* win);
    void PrintWindowTitle(WINDOW* win, int title_center, const char* title);
    void SetupTopWindow(WINDOW* win);
    void SetupDVLWindow(WINDOW* win);
    void SetupOdomWindow(WINDOW* win);
    void SetupBarometerWindow(WINDOW* win);
    void SetupIMUWindow(WINDOW* win);
    void SetupStateWindow(WINDOW* win);

    //void UpdateTopWindow(WINDOW* win);
    void UpdateDVLWindow(WINDOW* win, const brov2_interfaces::msg::DVL::ConstSharedPtr msg);
    void UpdateOdomWindow(WINDOW* win, const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg);
    void UpdateBarometerWindow(WINDOW* win, const brov2_interfaces::msg::Barometer::ConstSharedPtr msg);
    void UpdateIMUWindow(WINDOW* win, const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void UpdateStateWindow(WINDOW* win, const nav_msgs::msg::Odometry::ConstSharedPtr msg);//, bluerov_interfaces::msg::Reference latest_reference);
    
    void SetupTui(std::vector<WINDOW*>);


private:
    const int TOP_WINDOW_HEIGHT = 8;
    const int DATA_PLACEMENT = 50;
    const int UOM_PLACEMENT = 70;

    const int SENSOR1_WINDOW_HEIGHT = 13;
    const int SENSOR1_WINDOW_WIDTH = 84;
    const int SENSOR1_ROW_PLACEMENT = TOP_WINDOW_HEIGHT;
    const int SENSOR1_COL_PLACEMENT = 0;

    const int SENSOR2_WINDOW_HEIGHT = 13;
    const int SENSOR2_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR2_ROW_PLACEMENT = SENSOR1_ROW_PLACEMENT + SENSOR1_WINDOW_HEIGHT;
    const int SENSOR2_COL_PLACEMENT = 0;

    const int SENSOR3_WINDOW_HEIGHT = 10;
    const int SENSOR3_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR3_ROW_PLACEMENT = SENSOR2_ROW_PLACEMENT + SENSOR2_WINDOW_HEIGHT;
    const int SENSOR3_COL_PLACEMENT = 0;
    

    const int SENSOR4_WINDOW_HEIGHT = 16;
    const int SENSOR4_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR4_ROW_PLACEMENT = TOP_WINDOW_HEIGHT;
    const int SENSOR4_COL_PLACEMENT = SENSOR1_WINDOW_WIDTH;

    const int SENSOR5_WINDOW_HEIGHT = 20;
    const int SENSOR5_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR5_ROW_PLACEMENT = SENSOR4_ROW_PLACEMENT + SENSOR4_WINDOW_HEIGHT;
    const int SENSOR5_COL_PLACEMENT = SENSOR2_WINDOW_WIDTH;

};



#endif /* NCURSES_FUNCTIONS */