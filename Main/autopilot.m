function y = autopilot(uu, AP)

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
    alpha    = uu(5+NN);  % angle of attack
    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    Vg       = uu(13+NN); % ground speed
    wn       = uu(14+NN); % wind North
    we       = uu(15+NN); % wind East
    psi      = uu(16+NN); % heading
    bx       = uu(17+NN); % x-gyro bias
    by       = uu(18+NN); % y-gyro bias
    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;

    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    phi_c    = 0;
    theta_c  = 0;
    NN = NN+3;
    t        = uu(1+NN);   % time
    




    %---------------------------------Lateral Autopilot-----------------------------
    if t==0
        phi_c   = course_with_roll(chi_c, chi, r, 1, AP);
        delta_a = roll_with_aileron(phi_c, phi, p, 1, AP);
        delta_r = 0;%sideslip_with_rudder(beta, 1, AP);
    else
        phi_c   = course_with_roll(chi_c, chi, r, 0, AP);
        delta_a = roll_with_aileron(phi_c, phi, p, 0, AP);
        delta_r = 0;%sideslip_with_rudder(beta, 0, AP);
    end
    
    
    
    %----------------------------Longitudinal Autopilot--------------------------
    if t==0
        delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
        theta_c = altitude_with_pitch(h_c, h, 1, AP);
        %theta_c = airspeed_with_pitch(Va_c, Va, 1, AP);
        delta_e =  pitch_with_elevator(theta_c, theta, q, 1, AP);

    else
        delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
        theta_c = altitude_with_pitch(h_c, h, 0, AP);
        %theta_c = airspeed_with_pitch(Va_c, Va, 0, AP);
        delta_e = pitch_with_elevator(theta_c, theta, q, 0, AP);
    end


% persistent altitude_state;
% if h <= h_c - AP.altitude_zone
%     altitude_state = 2;
% 
% elseif h >= h_c + AP.altitude_zone
%     altitude_state = 3;
% 
% else
%     altitude_state = 4;
% end
%     
% if t==0
%     delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
%     theta_c = altitude_with_pitch(h_c, h, 1, AP);
%     theta_c = airspeed_with_pitch(Va_c, Va, 1, AP);      
% else
%     switch altitude_state
%         case 1  % in take-off zone
%             delta_t = 0;
%             theta_c = 0;
%                 
%         case 2  % climb zone
%             delta_t = 1;
%             theta_c = airspeed_with_pitch(Va_c, Va, 0, AP);
%                 
%         case 3 % descend zone
%             delta_t = 0;
%             theta_c = airspeed_with_pitch(Va_c, Va, 0, AP);
%                 
%         case 4 % altitude hold zone
%             delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
%             theta_c = altitude_with_pitch(h_c, h, 0, AP);
%     end
% end
% 
% 
% if t==0
%     delta_e = pitch_with_elevator(theta_c, theta, q, 1, AP);
% else
%     delta_e = pitch_with_elevator(theta_c, theta, q, 0, AP);
% end


    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];

    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...              % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------Regulate Roll Using The Aileron-----------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_with_aileron(phi_c, phi, p, flag, AP)

    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end
 
    error = phi_c - phi;

    integrator = integrator + (AP.Ts/2)*(error + error_d1);

    up = AP.roll_kp * error;
    ui = AP.roll_ki * integrator;
    ud = -AP.roll_kd * p;

    delta_a = sat(up + ui + ud, AP.delta_a_max, -AP.delta_a_max);

    if AP.roll_ki~=0
        delta_a_unsat = up + ui + ud;
        integrator = integrator + (AP.Ts/AP.roll_ki)*(delta_a - delta_a_unsat);
    end

    error_d1 = error;
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------Regulate Heading Using The Roll--------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c_sat = course_with_roll(chi_c, chi, r, flag, AP)

    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end
    
    error = chi_c - chi;

    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule
  
    differentiator = (2*AP.time_cons_course - AP.Ts)/(2*AP.time_cons_course + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_course + AP.Ts))*(error - error_d1);

    up = AP.course_kp * error;
    ui = AP.course_ki * integrator;
    ud = -AP.course_kd * differentiator;

    phi_c_sat = sat(up + ui + ud, AP.phi_max, -AP.phi_max);
  
    if AP.course_ki~=0
        phi_c_unsat = up + ui + ud;
        integrator = integrator + (AP.Ts/AP.course_ki)*(phi_c_sat - phi_c_unsat);
    end

    error_d1 = error;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------Sideslip With Rudder---------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = sideslip_with_rudder(v, flag, AP)
    
    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end

    error = 0 - v;

    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule

    differentiator = (2*AP.time_cons_beta - AP.Ts)/(2*AP.time_cons_beta + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_beta + AP.Ts))*(error - error_d1);

    up = -AP.sideslip_kp * v;
    ui = -AP.sideslip_ki * v;
    ud = -AP.sideslip_kd * differentiator;

    delta_r = sat(up + ui + ud, AP.delta_r_max, -AP.delta_r_max);

    error_d1 = error;

end












%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------Regulate Pitch Using Elevator-------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_with_elevator(theta_c, theta, q, flag, AP)

    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end
    
    error = theta_c - theta;

    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule
  
    differentiator = (2*AP.time_cons_theta - AP.Ts)/(2*AP.time_cons_theta + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_theta + AP.Ts))*(error - error_d1);

    up = AP.pitch_kp * error;
    ui = AP.pitch_ki * integrator;
    ud = -AP.pitch_kd * q;

    delta_e = sat(up + ui + ud, AP.delta_e_max, -AP.delta_e_max);
  
    if AP.pitch_ki~=0
        delta_e_unsat = up + ui + ud;
        integrator = integrator + (AP.Ts/AP.pitch_ki)*(delta_e - delta_e_unsat);
    end

    error_d1 = error;


end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------- Regulate Airspeed Using Throttle--------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP)

    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end

    Va_trim  = 0.3143;
    
    error = Va_c - Va;

    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule
  
    differentiator = (2*AP.time_cons_v - AP.Ts)/(2*AP.time_cons_v + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_v + AP.Ts))*(error - error_d1);

    up = AP.airspeed_throttle_kp * error;
    ui = AP.airspeed_throttle_ki * integrator;
    ud = AP.airspeed_throttle_kd * differentiator;

    delta_t_sat = sat(Va_trim + up + ui + ud, 1, 0.1);
  
    if AP.airspeed_throttle_ki~=0
        delta_t_unsat = up + ui + ud;
        integrator = integrator + (AP.Ts/AP.airspeed_throttle_ki)*(delta_t_sat - delta_t_unsat);
    end

    error_d1 = error;


end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------Regulate Airspeed Using Pitch Angle-------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch(Va_c, Va, flag, AP)
    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end
 
    error = Va_c - Va;
  
    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule
  
    differentiator = (2*AP.time_cons_v2 - AP.Ts)/(2*AP.time_cons_v2 + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_v2 + AP.Ts))*(error - error_d1);

    up = -AP.airspeed_pitch_kp * error
    ui = AP.airspeed_pitch_ki * integrator
    ud = AP.airspeed_pitch_kd * differentiator;
  
    theta_c = sat(up + ui + ud, AP.theta_max, -AP.theta_max);
  
    if AP.airspeed_pitch_ki~=0
        theta_c_unsat = up + ui + ud;
        integrator = integrator + AP.Ts/AP.airspeed_pitch_ki * (theta_c - theta_c_unsat);
    end

    error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------Regulate Altitude Using Pitch Angle-----------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)

    persistent integrator;
    persistent differentiator;
    persistent error_d1;

    if flag==1
        integrator = 0; 
        differentiator = 0;
        error_d1   = 0; 
    end

    error = h_c - h;

    integrator = integrator + (AP.Ts/2)*(error + error_d1); % trapazoidal rule
  
    differentiator = (2*AP.time_cons_h - AP.Ts)/(2*AP.time_cons_h + AP.Ts)*differentiator...
        + (2/(2*AP.time_cons_h + AP.Ts))*(error - error_d1);

    up = AP.altitude_kp * error;
    ui = AP.altitude_ki * integrator;
    ud = AP.altitude_kd * differentiator;

    theta_c_sat = sat(up + ui + ud, AP.theta_max, -AP.theta_max);
  
    if AP.altitude_ki~=0
        theta_c_unsat = up + ui + ud;
        integrator = integrator + (AP.Ts/AP.altitude_ki)*(theta_c_sat - theta_c_unsat);
    end

    error_d1 = error;

end










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
    if in >= up_limit
        out = up_limit;
    elseif in <= low_limit
        out = low_limit;
    else
        out = in;
    end
end
 