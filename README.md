# phidget_ros_drivers

## PREFORM CALIBRATION
Connect load cell to specific channel of PhigetBridge. Let node run to determin tare (around 5seconds). follow instrunctions (place known weight on loadcell and wait for stabile measurement). Adjust tolerance in launch file
## ONCE CALIBRATION IS DONE
Add values to loadcell_calibrations.yaml according to exisiting structure
## GETTING MEASUREMENTS
Adjust name of loadcell in launch file on given channel (coresponds to index in array of names) 
