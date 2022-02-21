#Estimate of roll from DVL
float64 roll
#Estimate of pitch from DVL
float64 pitch
#Estimate of yaw from DVL
float64 yaw
#Distance in X direction (m)
float64 x
#Distance in Y direction (m)
float64 y
#Distance in downward direction (m)
float64 z
#Status, 0 means no errors
int64 status
#Standard deviation (Figure of merit) for position
float64 std
#Time stamp (milliseconds since last press of reset button)
float64 ts
#Formatting of json
string form
#Report type
string type