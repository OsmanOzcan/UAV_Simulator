
function xhat = estimate_states(uu, state, MAV, SENSOR, SIM)

    y_gyro_x      = uu(1);
    y_gyro_y      = uu(2);
    y_gyro_z      = uu(3);
    y_accel_x     = uu(4);
    y_accel_y     = uu(5);
    y_accel_z     = uu(6);
    y_static_pres = uu(7);
    y_diff_pres   = uu(8);
    y_gps_n       = uu(9);
    y_gps_e       = uu(10);
    y_gps_h       = uu(11);
    y_gps_Vg      = uu(12);
    y_gps_course  = uu(13);
    t             = uu(14);
    u = state(4);
    v = state(5);
    w = state(6);
   
    % define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_x   % low pass filter of x-gyro
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent lpf_gyro_z   % low pass filter of z-gyro
    persistent lpf_static   % low pass filter of static pressure sensor
    persistent lpf_diff     % low pass filter of diff pressure sensor
    persistent lpf_accel_x  % low pass filter of x-accelerometer
    persistent lpf_accel_y  % low pass filter of y-accelerometer
    persistent lpf_accel_z  % low pass filter of z-accelerometer
    persistent xhat_a       % estimate of roll and pitch
    persistent phihat
    persistent thetahat
    persistent P_a          % error covariance for roll and pitch angles
    persistent xhat_p       % estimate of pn, pe, Vg, chi, wn, we, psi
    persistent P_p          % error covariance for pn, pe, Vg, chi, wn, we, psi
    persistent y_gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent y_gps_e_old  % last measurement of gps_e - used to detect new GPS signal
    persistent y_gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    persistent y_gps_course_old  % last measurement of gps_course - used to detect new GPS signal
    persistent pnhat
    persistent pehat
    persistent Vghat
    persistent chihat
    persistent wnhat
    persistent wehat
    persistent psihat
    persistent u_old
    persistent v_old
    persistent w_old

    lpf_a = 20;
    if t==0
        alpha        = exp(-lpf_a*SIM.ts_simulation);   % 0.67 for 0.02s ts
        lpf_gyro_x   = 0;
        lpf_gyro_y   = 0;
        lpf_gyro_z   = 0;
        lpf_static   = 0;
        lpf_diff     = 0;
        lpf_accel_x  = 0;
        lpf_accel_y  = 0;
        lpf_accel_z  = 0;
        xhat_a       = [0; 0];
        phihat       = 0;
        thetahat     = 0;
        P_a          = eye(2)*0.002;
        xhat_p       = [0;0;25;0;0;0;0];
        P_p          = zeros(7);
        y_gps_n_old  = -9999;
        y_gps_e_old  = -9999;
        y_gps_Vg_old = -9999;
        y_gps_course_old  = -9999;


        pnhat    = xhat_p(1);
        pehat    = xhat_p(2);
        Vghat    = xhat_p(3);
        chihat   = xhat_p(4); 
        wnhat    = xhat_p(5);
        wehat    = xhat_p(6);
        psihat   = xhat_p(7);

        u_old = MAV.u0;
        v_old = MAV.v0;
        w_old = MAV.w0;
    end


    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOW PASS FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %-------------------Low pass filter gyros to estimate angular rates---------------------
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*y_gyro_z;

    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;


    %----------------------------Low pass filter accelerometer------------------------------
    lpf_accel_x = alpha*lpf_accel_x + (1-alpha)*y_accel_x;
    lpf_accel_y = alpha*lpf_accel_y + (1-alpha)*y_accel_y;
    lpf_accel_z = alpha*lpf_accel_z + (1-alpha)*y_accel_z;

    
    %--------------------Low pass filter static pressure to estimate altitude---------------
    lpf_static = alpha*lpf_static + (1-alpha)*y_static_pres;
    hhat = lpf_static/(MAV.rho*MAV.gravity);
    

    %-----------------------Low pass filter diff pressure to estimate Va--------------------
    lpf_diff = alpha*lpf_diff + (1-alpha)*y_diff_pres;
    Vahat = sqrt(2/MAV.rho*lpf_diff);
    

    
    eulerAngles = Quaternion2Euler(state(7:10));
    phi      = eulerAngles(1);       % roll angle         
    theta    = eulerAngles(2);       % pitch angle     
    psi      = eulerAngles(3);       % yaw angle

    




    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %-----------Implement continous-discrete EKF to estimate roll and pitch angles-----------
    Q_a = eye(2)*0.0000000001;  % tune filter with this variable
    R_accel = eye(3)*SENSOR.accel_sigma^2;
    
    N = 10;
    % prediction step

    for i=1:N
        xhat_a = xhat_a + (SIM.ts_simulation/N)*[phat+qhat*sin(xhat_a(1))*tan(xhat_a(2))+rhat*cos(xhat_a(1))*tan(xhat_a(2));...
                                                qhat*cos(xhat_a(1))-rhat*sin(xhat_a(1))];

        A_a = [qhat*cos(xhat_a(1))*tan(xhat_a(2))-rhat*sin(xhat_a(1))*tan(xhat_a(2)), (qhat*sin(xhat_a(1))-rhat*cos(xhat_a(1)))/(cos(xhat_a(2))^2);...
            -qhat*sin(xhat_a(1))-rhat*cos(xhat_a(1)), 0];

        P_a = P_a + (SIM.ts_simulation/N)*(A_a*P_a + P_a*(A_a') + Q_a);
    end

    % measurement updates
    if t>=0
        h_a =[(u-u_old)/SIM.ts_simulation+qhat*Vahat*sin(xhat_a(2))+MAV.gravity*sin(xhat_a(2));...
            (v-v_old)/SIM.ts_simulation+rhat*Vahat*cos(xhat_a(2))-phat*Vahat*sin(xhat_a(2))-MAV.gravity*cos(xhat_a(2))*sin(xhat_a(1));...
            (w-w_old)/SIM.ts_simulation-qhat*Vahat*cos(xhat_a(2))-MAV.gravity*cos(xhat_a(2))*cos(xhat_a(1))];
        
        C_a =[0, qhat*Vahat*cos(xhat_a(2))+MAV.gravity*cos(xhat_a(2));...
            -MAV.gravity*cos(xhat_a(2))*cos(xhat_a(1)), -rhat*Vahat*sin(xhat_a(2))-phat*Vahat*cos(xhat_a(2))+MAV.gravity*sin(xhat_a(2))*sin(xhat_a(1));...
            MAV.gravity*cos(xhat_a(2))*sin(xhat_a(1)), (qhat*Vahat+MAV.gravity*cos(xhat_a(1)))*sin(xhat_a(2))];
        
        L_a = P_a*(C_a')/(R_accel + C_a*P_a*(C_a'));
        P_a = (eye(2) - L_a*C_a)*P_a;

        y_accels = [lpf_accel_x; lpf_accel_y; lpf_accel_z];
        xhat_a = xhat_a + L_a*(y_accels - h_a);

    end
    
    phihat   = xhat_a(1);
    thetahat = xhat_a(2);




    
    %---------------Implement continous-discrete EKF to estimate pn, pe, chi, Vg--------------
    Q_p = eye(7)*10;
    R_p = eye(6)*[SENSOR.gps_n_sigma^2;...       % y_gps_n
                SENSOR.gps_e_sigma^2;...         % y_gps_e
                SENSOR.gps_Vg_sigma^2;...        % y_gps_Vg
                SENSOR.gps_course_sigma^2;...    % y_gps_course
                0.001;...                        % pseudo measurement #1
                0.001];
    
    N = 10;
    % prediction step
    for i=1:N
        
        psidot = qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat);

        xhat_p = xhat_p + (SIM.ts_simulation/N)*[xhat_p(3)*cos(xhat_p(4));...
                                xhat_p(3)*sin(xhat_p(4));...
                                ((Vahat*cos(xhat_p(7))+xhat_p(5))*(-Vahat*psidot*sin(xhat_p(7))) + (Vahat*sin(xhat_p(7))+xhat_p(6))*(Vahat*psidot*cos(xhat_p(7))))/xhat_p(3);...
                                MAV.gravity/xhat_p(3)*tan(phihat)*cos(xhat_p(4)-xhat_p(7));...
                                0;...
                                0;...
                                qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat)];
        
        dVgdotDPsi = -(psidot*Vahat*(xhat_p(5)*cos(xhat_p(7)) + xhat_p(6)*sin(xhat_p(7))))/xhat_p(3);
        dChidotDVg = -(MAV.gravity*tan(phihat)*cos(xhat_p(4)-xhat_p(7)))/(xhat_p(3)^2);
        dChidotDChi = -(MAV.gravity*tan(phihat)*sin(xhat_p(4)-xhat_p(7)))/xhat_p(3);
        dChidotDPsi = (MAV.gravity*tan(phihat)*sin(xhat_p(4)-xhat_p(7)))/xhat_p(3);

        VgDot = ((Vahat*cos(xhat_p(7))+xhat_p(5))*(-Vahat*psidot*sin(xhat_p(7))) + (Vahat*sin(xhat_p(7))+xhat_p(6))*(Vahat*psidot*cos(xhat_p(7))))/xhat_p(3);
        
        A_p=[0 0 cos(xhat_p(4)) -xhat_p(3)*sin(xhat_p(4)) 0 0 0;...
             0 0 sin(xhat_p(4))  xhat_p(3)*cos(xhat_p(4)) 0 0 0;...
             0 0 -(VgDot)/xhat_p(3) 0 -psidot*Vahat*sin(xhat_p(7)) psidot*Vahat*cos(xhat_p(7)) dVgdotDPsi;...
             0 0 dChidotDVg dChidotDChi 0 0 dChidotDPsi;...
             0 0 0 0 0 0 0;...
             0 0 0 0 0 0 0;...
             0 0 0 0 0 0 0];

        P_p = P_p + (SIM.ts_simulation/N)*(A_p*P_p + P_p*(A_p') + Q_p);
    end


    % measurement updates
    if ((y_gps_n~=y_gps_n_old) | (y_gps_e~=y_gps_e_old) | (y_gps_Vg~=y_gps_Vg_old) | (y_gps_course~=y_gps_course_old))

        h_p = [xhat_p(1); xhat_p(2); xhat_p(3); xhat_p(4); Vahat*cos(xhat_p(7)) + xhat_p(5) - xhat_p(3)*cos(xhat_p(4));...
            Vahat*sin(xhat_p(7)) + xhat_p(6) - xhat_p(3)*sin(xhat_p(4))]

        C_p = [1 0 0 0 0 0 0;...
               0 1 0 0 0 0 0;...
               0 0 1 0 0 0 0;...
               0 0 0 1 0 0 0;...
               0 0 -cos(xhat_p(4)) xhat_p(3)*sin(xhat_p(4)) 1 0 -Vahat*sin(xhat_p(7));...
               0 0 -sin(xhat_p(4)) -xhat_p(3)*cos(xhat_p(4)) 0 1  Vahat*cos(xhat_p(7))];

        L_p = P_p*(C_p')/(R_p + C_p*P_p*(C_p'));
        P_p = (eye(7) - L_p*C_p)*P_p;
        yGPS = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course; 0; 0]
        xhat_p = xhat_p + L_p*(yGPS - h_p);

        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;

    end


        pnhat    = xhat_p(1);
        pehat    = xhat_p(2);
        Vghat    = xhat_p(3);
        chihat   = xhat_p(4); 
        wnhat    = xhat_p(5);
        wehat    = xhat_p(6);
        psihat   = xhat_p(7);


  
    alphahat = 0;
    betahat = 0;

    

    
%     psihat = psi;
% 
%     pnhat = y_gps_n;
%     pehat = y_gps_e;
%     alphahat = 0;
%     betahat = 0;
%     chihat = y_gps_course;
%     Vghat = y_gps_Vg;
%     wnhat = 0;
%     wehat = 0;

    u_old = u;
    v_old = v;
    w_old = w;



    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        0;...
        0;...
        0];
end