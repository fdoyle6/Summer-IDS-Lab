# Summer-IDS-Lab - June 2021-Present

### GitHub Summary: 
The programs written over the summer of 2021 while working for the University of Delaware's 
Information and Decision Sciences (IDS) Lab. For more information about the lab and its other
projects, please visit https://sites.udel.edu/ids-lab/ .

### Detailed Repository Description: 
During the summer going into my senior year at UD, I created a set of programs to estimate the 
state of a car in the IDS Lab's Scaled Smart City (SSC) using only the vehicle's onboard
sensors. This ensures the car would be able to follow a preset trajectory in the city if it 
were to lose contact with VICON (an effective GPS system) and/or the centralized control system 
(CCS). This was done by using an Extended Kalman Filter, and the code in this repository was 
used to advance the existing code for the SSC cars. Note: Most of the code for the cars in
the SSC is in a private repository so the full programs will not be shown, though what my code
is in here will be the same form as it is in the hidden programs.

### File Descriptions:
  - Arduino_Code_Addition.cpp - A few lines of code to be added to the Arduino that performs 
  the low level controls for the system so it would record data from the sensors in the car
  (from a Zumo 32U4) and send it to the Raspberry Pi that wirelessly connects with the CCS.
  
  - Pi_Sensor_Script_Addition.py - The additions to the code that already runs on the RasPi
  during an experiment. This extra code allows the Pi to accept and save data from the Arduino
  and VICON while also exporting it to other files that will be analyzed later to calculate 
  the variances in each measurement. This code will later include the code to estimate the
  state of the system (and uncertainty) using the previously mentioned Extended Kalman Filter.
  
  - Variance_Calculator.py - The script used to calculate the uncertatainties (variances) of 
  each sensor on-board the car. This uses the files output by the Pi_Sensor_Script_Addition.py
  program additions. 
