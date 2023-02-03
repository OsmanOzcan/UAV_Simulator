load Datas/tf_coef_linePath
run('../parameters/simulation_parameters')   % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')    % load MAV: aircraft parameters

% AP stands for autopilot
AP.gravity = MAV.gravity;
AP.Vat = 25;
AP.Ts = SIM.ts_simulation;




%---------------------------------Roll Loop-----------------------------
[num,den]=tfdata(T_phi_delta_a,'v');
a_phi2 = num(3);
a_phi1 = den(2);

AP.delta_a_max = 45*pi/180;             % Maximum possible aileron command
AP.phi_max = 25*pi/180;                 % Roll command when delta_a_max is achieved

wn_roll = sqrt(abs(a_phi2)*AP.delta_a_max/AP.phi_max);
zeta_roll = 0.992;
AP.time_cons_roll = 1/(wn_roll*zeta_roll);

AP.roll_kp = AP.delta_a_max/AP.phi_max*sign(a_phi2);
AP.roll_kd = (2*zeta_roll*wn_roll-a_phi1)/a_phi2;
AP.roll_ki = 0.05;




%-----------------------------Course Loop------------------------------
wn_course = wn_roll/100;
zeta_course = 1.376;
AP.time_cons_course = 1/(wn_course*zeta_course);

AP.course_kp = 2*zeta_course*wn_course*AP.Vat/AP.gravity;
AP.course_ki = (wn_course^2)*AP.Vat/AP.gravity;
AP.course_kd = 0;




%----------------------------Sideslip Loop------------------------------
[numT,denT]=tfdata(T_v_delta_r,'v');
a_beta2 = numT(2);
a_beta1 = denT(2);

AP.delta_r_max = 35*pi/180;
AP.beta_max = 20*pi/180;

zeta_beta = 1;

AP.sideslip_kp = AP.delta_r_max/AP.beta_max*sign(a_beta2);
AP.sideslip_ki = (((a_beta1 + a_beta2*AP.sideslip_kp)/(2*zeta_beta))^2)/a_beta2;
AP.sideslip_kd = 0;


wn_beta = (a_beta1 + a_beta2*AP.sideslip_kp)/(2*zeta_beta);
AP.time_cons_beta = 1/(wn_beta*zeta_beta);





%-----------------------------Pitch Loop--------------------------------
[numPAH,denPAH]=tfdata(T_theta_delta_e,'v');
a_theta3=numPAH(3);
a_theta2=denPAH(3);
a_theta1=denPAH(2);

AP.delta_e_max = 45*pi/180;
AP.theta_max = 15*pi/180;

wn_theta = sqrt(a_theta2 + (AP.delta_e_max/AP.theta_max)*abs(a_theta3));
zeta_theta = 0.789;
AP.time_cons_theta = 1/(wn_theta*zeta_theta);

AP.pitch_kp = AP.delta_e_max/(AP.theta_max)*sign(a_theta3);
AP.pitch_kd = (2*zeta_theta*wn_theta - a_theta1)/a_theta3;
AP.pitch_ki = 0.0;
AP.K_theta_DC = AP.pitch_kp*a_theta3/(a_theta2 + AP.pitch_kp*a_theta3);




%----------------------------Altitude Loop---------------------------
wn_h = wn_theta/20;
zeta_h = 0.877;
AP.time_cons_h = 1/(wn_h*zeta_h);

AP.altitude_kp = 2*zeta_h*wn_h/(AP.K_theta_DC*AP.Vat);
AP.altitude_ki = wn_h^2/(AP.K_theta_DC*AP.Vat);
AP.altitude_kd = 0;
AP.altitude_zone = 5;





%---------------------Airspeed Hold Using Pitch---------------------
wn_v2 = wn_theta/10;
zeta_v2 = 0.907;
AP.time_cons_v2 = 1/(wn_v2*zeta_v2);

[numAHP,denAHP] = tfdata(T_Va_theta,'v');
a_V1 = denAHP(2);

AP.airspeed_pitch_kp = (a_V1 - 2*zeta_v2*wn_v2)/(AP.K_theta_DC*AP.gravity);
AP.airspeed_pitch_ki = -(wn_v2^2)/(AP.K_theta_DC*AP.gravity);
AP.airspeed_pitch_kd = 0;





%---------------------Airspeed Hold Using Throttle--------------------
wn_v = wn_v2*8;
zeta_v = 0.974;
AP.time_cons_v = 1/(wn_v*zeta_v);

[numAHT,denAHT] = tfdata(T_Va_delta_t,'v');
a_V2 = numAHT(2);
   
AP.airspeed_throttle_kp = (2*zeta_v*wn_v - a_V1)/((a_V2));
AP.airspeed_throttle_ki = wn_v^2/(a_V2);
AP.airspeed_throttle_kd = 0;


