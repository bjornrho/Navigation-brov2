std_msgs/Header header
#Time
float64 time
#Measured velocity [m/s]
geometry_msgs/Vector3 velocity
#Figure of merit, measure of the accuracy of the measured velocities (m/s)
float64 fom
#Covariance matrix for the velocities (entries in (m/s)^2)
float64[9] covariance
#Measured altitude to the bottom (m)
float64 altitude
#Beams/Transducers
DVLBeam[4] beams
#Validity of velocity
bool velocity_valid
#Status message
int64 status
#Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)
float64 time_of_validity
#Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)
float64 time_of_transmission
#Formatting of json
string form
#Report type
string type
