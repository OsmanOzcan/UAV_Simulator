function y = sensors(uu, MAV, SENSOR)

    % relabel the inputs
    pn      = uu(1);
    pe      = uu(2);
    pd      = uu(3);
    u       = uu(4);
    v       = uu(5);
    w       = uu(6);
    e0      = uu(7);
    e1      = uu(8);
    e2      = uu(9);
    e3      = uu(10);
    p       = uu(11);
    q       = uu(12);
    r       = uu(13);
    F_x     = uu(14);
    F_y     = uu(15);
    F_z     = uu(16);
    M_l     = uu(17);
    M_m     = uu(18);
    M_n     = uu(19);
    Va      = uu(20);
    alpha   = uu(21);
    beta    = uu(22);
    wn      = uu(23);
    we      = uu(24);
    wd      = uu(25);


    eulerAngles = Quaternion2Euler([e0 e1 e2 e3]);
    phi      = eulerAngles(1);       % roll angle         
    theta    = eulerAngles(2);       % pitch angle     
    psi      = eulerAngles(3);       % yaw angle 
    


    y_gyro_x = p + SENSOR.gyro_sigma*randn(1);
    y_gyro_y = q + SENSOR.gyro_sigma*randn(1);
    y_gyro_z = r + SENSOR.gyro_sigma*randn(1);

    y_accel_x = F_x/MAV.mass + MAV.gravity*sin(theta) + SENSOR.accel_sigma*randn(1);
    y_accel_y = F_y/MAV.mass - MAV.gravity*cos(theta)*sin(phi) + SENSOR.accel_sigma*randn(1);
    y_accel_z = F_z/MAV.mass - MAV.gravity*cos(theta)*cos(phi) + SENSOR.accel_sigma*randn(1);

    y_static_pres = MAV.rho*MAV.gravity*(-pd) + SENSOR.static_pres_sigma*randn(1);
    y_diff_pres =  MAV.rho*Va^2/2 + SENSOR.diff_pres_sigma*randn(1);



    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



