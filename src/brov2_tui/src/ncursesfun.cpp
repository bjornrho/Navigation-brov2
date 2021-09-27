#include "brov2_tui/ncursesfun.h"



NcursesFunctions::NcursesFunctions()
{
    initscr();
    has_colors();
}

NcursesFunctions::~NcursesFunctions()
{
    endwin();
}


std::vector<WINDOW*> NcursesFunctions::InitWindows()
{
    std::vector<WINDOW*> my_wins = {};

    WINDOW* top_window = newwin(TOP_WINDOW_HEIGHT, COLS, 0, 0);
    WINDOW* sensor_window_1 = newwin(SENSOR1_WINDOW_HEIGHT, SENSOR1_WINDOW_WIDTH, SENSOR1_ROW_PLACEMENT, SENSOR1_COL_PLACEMENT);
    WINDOW* sensor_window_2 = newwin(SENSOR2_WINDOW_HEIGHT, SENSOR2_WINDOW_WIDTH, SENSOR2_ROW_PLACEMENT, SENSOR2_COL_PLACEMENT);
    WINDOW* sensor_window_3 = newwin(SENSOR3_WINDOW_HEIGHT, SENSOR3_WINDOW_WIDTH, SENSOR3_ROW_PLACEMENT, SENSOR3_COL_PLACEMENT);
    WINDOW* sensor_window_4 = newwin(SENSOR4_WINDOW_HEIGHT, SENSOR4_WINDOW_WIDTH, SENSOR4_ROW_PLACEMENT, SENSOR4_COL_PLACEMENT);
    WINDOW* sensor_window_5 = newwin(SENSOR5_WINDOW_HEIGHT, SENSOR5_WINDOW_WIDTH, SENSOR5_ROW_PLACEMENT, SENSOR5_COL_PLACEMENT);

    my_wins.push_back(top_window);
    my_wins.push_back(sensor_window_1);
    my_wins.push_back(sensor_window_2);
    my_wins.push_back(sensor_window_3);
    my_wins.push_back(sensor_window_4);
    my_wins.push_back(sensor_window_5);

    return my_wins;
}

// void NcursesFunctions::DrawTopWindow(WINDOW* win)
// {
//     refresh();
//     wmove(win, TOP_WINDOW_HEIGHT, 0);
//     whline(win, '_', COLS);
//     wrefresh(win);
// }

void NcursesFunctions::InitColors()
{
    start_color();
    use_default_colors();
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    init_pair(2, COLOR_WHITE, COLOR_BLACK);
}

void NcursesFunctions::DrawBox(WINDOW* win)
{
    refresh();
    box(win, 0, 0);
    wrefresh(win);
}

void NcursesFunctions::PrintWindowTitle(WINDOW* win, int title_center, const char* title)
{
    wattron(win, COLOR_PAIR(1));
    mvwprintw(win, 0, title_center, title);
    wattroff(win, COLOR_PAIR(1));
}




//////////////////////////////////////////////
// Setup functions for the different sensors//
//////////////////////////////////////////////
void NcursesFunctions::SetupTopWindow(WINDOW* win)
{
    refresh();
    const char* title = "Visualization of sensor data";
    int title_center = round(COLS/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);
    wrefresh(win);
}    

void NcursesFunctions::SetupDVLWindow(WINDOW* win)
{
    refresh();
    const char* title = "DVL";
    int title_center = round(SENSOR1_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);

    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, 2, 1, "Velocity X");
    mvwprintw(win, 3, 1, "Velocity Y");
    mvwprintw(win, 4, 1, "Velocity Z");
    mvwprintw(win, 5, 1, "Validity of vel");
    mvwprintw(win, 6, 1, "Altitude");
    mvwprintw(win, 7, 1, "Figure of merit");
    for (int i = 2; i <= 7; i++){mvwprintw(win, i, SENSOR1_WINDOW_WIDTH/2, ":");}
    wattroff(win, COLOR_PAIR(2));

    wrefresh(win);
}

void NcursesFunctions::SetupOdomWindow(WINDOW* win)
{
    refresh();
    const char* title = "Odometry estimate - DVL";
    int title_center = round(SENSOR2_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);

    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, 2, 1, "Position X");
    mvwprintw(win, 3, 1, "Position Y");
    mvwprintw(win, 4, 1, "Position Z");
    mvwprintw(win, 5, 1, "Roll");
    mvwprintw(win, 6, 1, "Pitch");
    mvwprintw(win, 7, 1, "Yaw");
    mvwprintw(win, 8, 1, "Figure of merit");
    for (int i = 2; i <= 8; i++){mvwprintw(win, i, SENSOR2_WINDOW_WIDTH/2, ":");}
    wattroff(win, COLOR_PAIR(2));

    wrefresh(win);
}

void NcursesFunctions::SetupBarometerWindow(WINDOW* win){
    refresh();
    const char* title = "Barometer";
    int title_center = round(SENSOR3_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);

    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, 2, 1, "Depth");mvwprintw(win, 2, SENSOR3_WINDOW_WIDTH/2, ":");
    mvwprintw(win, 4, 1, "Pressure");mvwprintw(win, 4, SENSOR3_WINDOW_WIDTH/2, ":");
    mvwprintw(win, 6, 1, "Temperature");mvwprintw(win, 6, SENSOR3_WINDOW_WIDTH/2, ":");
    wattroff(win, COLOR_PAIR(2));
    
    wrefresh(win);
}

void NcursesFunctions::SetupIMUWindow(WINDOW* win)
{
    refresh();
    const char* title = "IMU";
    int title_center = round(SENSOR3_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);
    wrefresh(win);

}

void NcursesFunctions::SetupSonarWindow(WINDOW* win)
{
    refresh();
    const char* title = "Sonar";
    int title_center = round(SENSOR3_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);
    wrefresh(win);
}

void NcursesFunctions::SetupTui(std::vector<WINDOW*> my_wins)
{
    // Initialize colors and draw boxes around all windows
    InitColors();
    for (WINDOW* win : my_wins){
        DrawBox(win);
    }

    // Setting up every sensor window
    SetupTopWindow(my_wins[0]);
    SetupDVLWindow(my_wins[1]);
    SetupOdomWindow(my_wins[2]);
    SetupBarometerWindow(my_wins[3]);
    SetupIMUWindow(my_wins[4]);
    SetupSonarWindow(my_wins[5]);
}



//////////////////////////////////////////////////////
// Update functions for the different sensor windows//
//////////////////////////////////////////////////////
void NcursesFunctions::UpdateBarometerWindow(WINDOW* win, const brov2_interfaces::msg::Barometer::ConstSharedPtr msg)
{
    refresh();
    wattron(win, COLOR_PAIR(2));
    //mvwprintw(win, 1, SENSOR3_DATA_PLACEMENT, ""+msg->depth);
    std::stringstream ss;
    ss << msg->pressure_mbar;
    const char* str = ss.str().c_str();
    mvwprintw(win, 3, SENSOR3_DATA_PLACEMENT, str);
    //mvwprintw(win, 5, SENSOR3_DATA_PLACEMENT, std::to_string(msg->temperature_celsius));
    wattroff(win, COLOR_PAIR(2));
    wrefresh(win);
}
