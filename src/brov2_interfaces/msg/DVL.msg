std_msgs/Header header
#Time
float64 time
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
#Figure of merit, measure of the accuracy of the measured velocities (m/s)
float64 fom
#Measured altitude to the bottom (m)
float64 altitude
#Beams/Transducers
DVLBeam[4] beams
#Validity of velocity
bool velocity_valid
#Status message
int64 status
#Formatting of json
string form