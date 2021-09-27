#ifndef NCURSESFUN_H
#define NCURSESFUN_H

#include <vector>
#include <ncurses.h>
#include <panel.h>
#include <string.h>
#include <cmath>

#include "brov2_interfaces/msg/barometer.hpp"


class NcursesFunctions
{
public:
    NcursesFunctions();
    ~NcursesFunctions();

    std::vector<WINDOW*> InitWindows();
    void InitColors();
    // void DrawTopWindow(WINDOW* win);
    void DrawBox(WINDOW* win);
    void PrintWindowTitle(WINDOW* win, int title_center, const char* title);
    void SetupTopWindow(WINDOW* win);
    void SetupDVLWindow(WINDOW* win);
    void SetupOdomWindow(WINDOW* win);
    void SetupBarometerWindow(WINDOW* win);
    void SetupIMUWindow(WINDOW* win);
    void SetupSonarWindow(WINDOW* win);
    void UpdateBarometerWindow(WINDOW* win, const brov2_interfaces::msg::Barometer::ConstSharedPtr msg);
    void SetupTui(std::vector<WINDOW*>);


private:
    const int TOP_WINDOW_HEIGHT = 8;

    const int SENSOR1_WINDOW_HEIGHT = 10;
    const int SENSOR1_WINDOW_WIDTH = 84;
    const int SENSOR1_ROW_PLACEMENT = TOP_WINDOW_HEIGHT;
    const int SENSOR1_COL_PLACEMENT = 0;

    const int SENSOR2_WINDOW_HEIGHT = 11;
    const int SENSOR2_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR2_ROW_PLACEMENT = SENSOR1_ROW_PLACEMENT + SENSOR1_WINDOW_HEIGHT;
    const int SENSOR2_COL_PLACEMENT = 0;

    const int SENSOR3_WINDOW_HEIGHT = 10;
    const int SENSOR3_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR3_ROW_PLACEMENT = SENSOR2_ROW_PLACEMENT + SENSOR2_WINDOW_HEIGHT;
    const int SENSOR3_COL_PLACEMENT = 0;
    const int SENSOR3_DATA_PLACEMENT = 30;

    const int SENSOR4_WINDOW_HEIGHT = 10;
    const int SENSOR4_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR4_ROW_PLACEMENT = TOP_WINDOW_HEIGHT;
    const int SENSOR4_COL_PLACEMENT = SENSOR1_WINDOW_WIDTH;

    const int SENSOR5_WINDOW_HEIGHT = 21;
    const int SENSOR5_WINDOW_WIDTH = SENSOR1_WINDOW_WIDTH;
    const int SENSOR5_ROW_PLACEMENT = SENSOR4_ROW_PLACEMENT + SENSOR4_WINDOW_HEIGHT;
    const int SENSOR5_COL_PLACEMENT = SENSOR2_WINDOW_WIDTH;

};



#endif /* NCURSESFUN */