#include "brov2_tui/ncurses_functions.h"



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

void NcursesFunctions::InitColors()
{
    start_color();
    use_default_colors();
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    init_pair(2, COLOR_WHITE, COLOR_BLACK);
    init_pair(3, COLOR_RED, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);
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
    mvwprintw(win, 2, 1, "Velocity X");mvwprintw(win, 2, UOM_PLACEMENT, "[m/s]");
    mvwprintw(win, 3, 1, "Velocity Y");mvwprintw(win, 3, UOM_PLACEMENT, "[m/s]");
    mvwprintw(win, 4, 1, "Velocity Z");mvwprintw(win, 4, UOM_PLACEMENT, "[m/s]");
    mvwprintw(win, 6, 1, "Altitude");mvwprintw(win, 6, UOM_PLACEMENT, "[m]");
    mvwprintw(win, 8, 1, "Validity of vel");
    mvwprintw(win, 9, 1, "Figure of merit");mvwprintw(win, 9, UOM_PLACEMENT, "[m/s]");
    mvwprintw(win, 11, 1, "Temperature status");
    for (int i = 2; i <=11; i++){
        if (i != 5 && i != 7 && i != 10){mvwprintw(win, i, SENSOR1_WINDOW_WIDTH/2, ":");}}
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
    mvwprintw(win, 2, 1, "Position X");mvwprintw(win, 2, UOM_PLACEMENT, "[m]");
    mvwprintw(win, 3, 1, "Position Y");mvwprintw(win, 3, UOM_PLACEMENT, "[m]");
    mvwprintw(win, 4, 1, "Position Z");mvwprintw(win, 4, UOM_PLACEMENT, "[m]");
    mvwprintw(win, 6, 1, "Roll");mvwprintw(win, 6, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 7, 1, "Pitch");mvwprintw(win, 7, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 8, 1, "Yaw");mvwprintw(win, 8, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 10, 1, "Figure of merit");mvwprintw(win, 10, UOM_PLACEMENT, "[m]");

    for (int i = 2; i <=10; i++){
        if (i != 5 && i != 9){mvwprintw(win, i, SENSOR2_WINDOW_WIDTH/2, ":");}}
    wattroff(win, COLOR_PAIR(2));

    wrefresh(win);
}

void NcursesFunctions::SetupBarometerWindow(WINDOW* win){
    refresh();
    const char* title = "Barometer";
    int title_center = round(SENSOR3_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);

    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, 2, 1, "Depth");mvwprintw(win, 2, SENSOR3_WINDOW_WIDTH/2, ":");mvwprintw(win, 2, UOM_PLACEMENT, "[m]");
    mvwprintw(win, 4, 1, "Pressure");mvwprintw(win, 4, SENSOR3_WINDOW_WIDTH/2, ":");mvwprintw(win, 4, UOM_PLACEMENT, "[mbar]");
    mvwprintw(win, 6, 1, "Temperature");mvwprintw(win, 6, SENSOR3_WINDOW_WIDTH/2, ":");mvwprintw(win, 6, UOM_PLACEMENT, "[celsius]");
    wattroff(win, COLOR_PAIR(2));
    
    wrefresh(win);
}

void NcursesFunctions::SetupIMUWindow(WINDOW* win)
{
    refresh();
    const char* title = "IMU";
    int title_center = round(SENSOR3_WINDOW_WIDTH/2) - round(strlen(title)/2);
    
    PrintWindowTitle(win, title_center, title);

    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, 2, 1, "Orientation X");mvwprintw(win, 2, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 3, 1, "Orientation Y");mvwprintw(win, 3, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 4, 1, "Orientation Z");mvwprintw(win, 4, UOM_PLACEMENT, "[rad]");
    mvwprintw(win, 5, 1, "Orientation W");mvwprintw(win, 5, UOM_PLACEMENT, "[rad]");

    mvwprintw(win, 7, 1, "Angular velocity X");mvwprintw(win, 7, UOM_PLACEMENT, "[rad/s]");
    mvwprintw(win, 8, 1, "Angular velocity Y");mvwprintw(win, 8, UOM_PLACEMENT, "[rad/s]");
    mvwprintw(win, 9, 1, "Angular velocity Z");mvwprintw(win, 9, UOM_PLACEMENT, "[rad/s]");

    mvwprintw(win, 11, 1, "Linear acceleration X");mvwprintw(win, 11, UOM_PLACEMENT, "[m/s^2]");
    mvwprintw(win, 12, 1, "Linear acceleration Y");mvwprintw(win, 12, UOM_PLACEMENT, "[m/s^2]");
    mvwprintw(win, 13, 1, "Linear acceleration Z");mvwprintw(win, 13, UOM_PLACEMENT, "[m/s^2]");

    for (int i = 2; i <=13; i++){
        if (i != 6 && i != 10){mvwprintw(win, i, SENSOR2_WINDOW_WIDTH/2, ":");}}
    wattroff(win, COLOR_PAIR(2));


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

//void NcursesFunctions::UpdateTopWindow(WINDOW* win)
//{}

void NcursesFunctions::UpdateDVLWindow(WINDOW* win, const brov2_interfaces::msg::DVL::ConstSharedPtr msg)
{
    // Initializing streams for each data container
    std::stringstream ss_x;
    std::stringstream ss_y;
    std::stringstream ss_z;
    std::stringstream ss_alt;
    std::stringstream ss_val;
    std::stringstream ss_fom;

    refresh();
    wattron(win, COLOR_PAIR(2));
    
    // Conversion before printing at correct place in window
    ss_x << msg->velocity.x;
    const char* vel_x_data = ss_x.str().c_str();
    mvwprintw(win, 2, DATA_PLACEMENT, vel_x_data);

    ss_y << msg->velocity.y;
    const char* vel_y_data = ss_y.str().c_str();
    mvwprintw(win, 3, DATA_PLACEMENT, vel_y_data);

    ss_z << msg->velocity.z;
    const char* vel_z_data = ss_z.str().c_str();
    mvwprintw(win, 4, DATA_PLACEMENT, vel_z_data);
    
    ss_alt << msg->altitude;
    const char* altitude_data = ss_alt.str().c_str();
    mvwprintw(win, 6, DATA_PLACEMENT, altitude_data);

    if (msg->velocity_valid){
        mvwprintw(win, 8, DATA_PLACEMENT, "   VALID   ");
    }else{
        mvwprintw(win, 8, DATA_PLACEMENT, " NOT VALID ");
    }

    ss_fom << msg->fom;
    const char* fom_data = ss_fom.str().c_str();
    mvwprintw(win, 9, DATA_PLACEMENT, fom_data);
    
    wattroff(win, COLOR_PAIR(2));

    if(msg->status == 1){
        wattron(win, COLOR_PAIR(3));
        mvwprintw(win, 11, DATA_PLACEMENT, "WARNING");
        wattroff(win, COLOR_PAIR(3));
    }else{
        wattron(win, COLOR_PAIR(4));
        mvwprintw(win, 11, DATA_PLACEMENT, "OK");
        wattroff(win, COLOR_PAIR(4));
    }
    
    wrefresh(win);
}

void NcursesFunctions::UpdateOdomWindow(WINDOW* win, const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg)
{
    // Initializing streams for each data container
    std::stringstream ss_x;
    std::stringstream ss_y;
    std::stringstream ss_z;
    std::stringstream ss_roll;
    std::stringstream ss_pitch;
    std::stringstream ss_yaw;
    std::stringstream ss_fom;

    refresh();
    wattron(win, COLOR_PAIR(2));

    // Conversion before printing at correct place in window
    ss_x << msg->x;
    const char* pos_x_data = ss_x.str().c_str();
    mvwprintw(win, 2, DATA_PLACEMENT, pos_x_data);

    ss_y << msg->y;
    const char* pos_y_data = ss_y.str().c_str();
    mvwprintw(win, 3, DATA_PLACEMENT, pos_y_data);

    ss_z << msg->z;
    const char* pos_z_data = ss_z.str().c_str();
    mvwprintw(win, 4, DATA_PLACEMENT, pos_z_data);

    ss_roll << msg->roll;
    const char* roll_data = ss_roll.str().c_str();
    mvwprintw(win, 6, DATA_PLACEMENT, roll_data);

    ss_pitch << msg->pitch;
    const char* pitch_data = ss_pitch.str().c_str();
    mvwprintw(win, 7, DATA_PLACEMENT, pitch_data);

    ss_yaw << msg->yaw;
    const char* yaw_data = ss_yaw.str().c_str();
    mvwprintw(win, 8, DATA_PLACEMENT, yaw_data);

    ss_fom << msg->std;
    const char* fom_data = ss_fom.str().c_str();
    mvwprintw(win, 10, DATA_PLACEMENT, fom_data);

    wattroff(win, COLOR_PAIR(2));
    wrefresh(win);
}

void NcursesFunctions::UpdateBarometerWindow(WINDOW* win, const brov2_interfaces::msg::Barometer::ConstSharedPtr msg)
{
    std::stringstream ss_d;
    std::stringstream ss_p;
    std::stringstream ss_t;
    refresh();
    wattron(win, COLOR_PAIR(2));
    
    ss_d << msg->depth;
    const char* depth_data = ss_d.str().c_str();
    mvwprintw(win, 2, DATA_PLACEMENT, depth_data);
    
    ss_p << msg->pressure_mbar;
    const char* pressure_data = ss_p.str().c_str();
    mvwprintw(win, 4, DATA_PLACEMENT, pressure_data);

    ss_t << msg->temperature_celsius;
    const char* temperature_data = ss_t.str().c_str();
    mvwprintw(win, 6, DATA_PLACEMENT, temperature_data);

    wattroff(win, COLOR_PAIR(2));
    wrefresh(win);
}

void NcursesFunctions::UpdateIMUWindow(WINDOW* win, const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    // Initializing streams for each data container
    std::stringstream ss_ori_x;
    std::stringstream ss_ori_y;
    std::stringstream ss_ori_z;
    std::stringstream ss_ori_w;
    std::stringstream ss_ang_x;
    std::stringstream ss_ang_y;
    std::stringstream ss_ang_z;
    std::stringstream ss_lin_x;
    std::stringstream ss_lin_y;
    std::stringstream ss_lin_z;

    refresh();
    wattron(win, COLOR_PAIR(2));

    // Conversion before printing at correct place in window
    ss_ori_x << msg->orientation.x;
    const char* ori_x_data = ss_ori_x.str().c_str();
    mvwprintw(win, 2, DATA_PLACEMENT, ori_x_data);

    ss_ori_y << msg->orientation.y;
    const char* ori_y_data = ss_ori_y.str().c_str();
    mvwprintw(win, 3, DATA_PLACEMENT, ori_y_data);

    ss_ori_z << msg->orientation.z;
    const char* ori_z_data = ss_ori_z.str().c_str();
    mvwprintw(win, 4, DATA_PLACEMENT, ori_z_data);

    ss_ori_w << msg->orientation.w;
    const char* ori_w_data = ss_ori_w.str().c_str();
    mvwprintw(win, 5, DATA_PLACEMENT, ori_w_data);

    ss_ang_x << msg->angular_velocity.x;
    const char* ang_x_data = ss_ang_x.str().c_str();
    mvwprintw(win, 7, DATA_PLACEMENT, ang_x_data);

    ss_ang_y << msg->angular_velocity.y;
    const char* ang_y_data = ss_ang_y.str().c_str();
    mvwprintw(win, 8, DATA_PLACEMENT, ang_y_data);

    ss_ang_z << msg->angular_velocity.z;
    const char* ang_z_data = ss_ang_z.str().c_str();
    mvwprintw(win, 9, DATA_PLACEMENT, ang_z_data);

    ss_lin_x << msg->linear_acceleration.x;
    const char* lin_x_data = ss_lin_x.str().c_str();
    mvwprintw(win, 11, DATA_PLACEMENT, lin_x_data);

    ss_lin_y << msg->linear_acceleration.y;
    const char* lin_y_data = ss_lin_y.str().c_str();
    mvwprintw(win, 12, DATA_PLACEMENT, lin_y_data);

    ss_lin_z << msg->linear_acceleration.z;
    const char* lin_z_data = ss_lin_z.str().c_str();
    mvwprintw(win, 13, DATA_PLACEMENT, lin_z_data);


    wattroff(win, COLOR_PAIR(2));
    wrefresh(win);
}
