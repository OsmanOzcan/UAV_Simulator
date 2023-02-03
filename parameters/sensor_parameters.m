%-------- Accelerometer --------
SENSOR.accel_sigma = 0.0025*9.8; 

%-------- Rate Gyro --------
SENSOR.gyro_x_bias = 0;  % bias on x_gyro
SENSOR.gyro_y_bias = 0;  % bias on y_gyro
SENSOR.gyro_z_bias = 0;  % bias on z_gyro
SENSOR.gyro_sigma = 0.23 * pi/180;  

%-------- Pressure Sensor(Altitude) --------
SENSOR.static_pres_sigma = 10;  % in Pascals

%-------- Pressure Sensor (Airspeed) --------
SENSOR.diff_pres_sigma = 2;   % in Pascals

%-------- Magnetometer --------
SENSOR.mag_beta = 1.0 * (pi/180);
SENSOR.mag_sigma = 0.03 * (pi/180);

%-------- GPS --------
SENSOR.gps_beta = 1 / 1100;  % 1 / s
SENSOR.gps_n_sigma = 2.21;
SENSOR.gps_e_sigma = 2.21;
SENSOR.gps_h_sigma = 5.40;
SENSOR.gps_Vg_sigma = 0.05;
SENSOR.gps_course_sigma = SENSOR.gps_Vg_sigma / 10;
