
function out = wind_simulation(state, WIND, sim_time)
    
    format long;

    u = state(4);
    v = state(5);
    w = state(6);

    eulerAngles = Quaternion2Euler(state(7:10));
    phi      = eulerAngles(1);       % roll angle         
    theta    = eulerAngles(2);       % pitch angle     
    psi      = eulerAngles(3);       % yaw angle 


    w_ns    = WIND.wind_n;          % steady wind - North
    w_es    = WIND.wind_e;          % steady wind - East
    w_ds    = WIND.wind_d;          % steady wind - Down
    

    % ---------------------------Dryden Wind Model---------------------------------

%     H_u = tf(WIND.sigma_u*sqrt(2*WIND.Va0/WIND.L_u), [1 WIND.Va0/WIND.L_u]);
%     H_v = tf(WIND.sigma_v*sqrt(3*WIND.Va0/WIND.L_v)*[1 WIND.Va0/(sqrt(3)*WIND.L_v)], poly([-WIND.Va0/WIND.L_v, -WIND.Va0/WIND.L_v]));
%     H_w = tf(WIND.sigma_w*sqrt(3*WIND.Va0/WIND.L_w)*[1, WIND.Va0/(sqrt(3)*WIND.L_w)], poly([-WIND.Va0/WIND.L_w, -WIND.Va0/WIND.L_w]));
%     
%     syms s t;
%     t = sim_time;
%     H_u_n = poly2sym(H_u.Numerator, s);
%     H_u_d = poly2sym(H_u.Denominator, s);
% 
%     H_v_n = poly2sym(H_v.Numerator, s);
%     H_v_d = poly2sym(H_v.Denominator, s);
% 
%     H_w_n = poly2sym(H_w.Numerator, s);
%     H_w_d = poly2sym(H_w.Denominator, s);
%     
%     u_wg = double(vpa(subs(ilaplace((randn(1)/s)*(H_u_n/H_u_d)))));     % gust along body x-axis
%     v_wg = double(vpa(subs(ilaplace((randn(1)/s)*(H_v_n/H_v_d)))));     % gust along body y-axis
%     w_wg = double(vpa(subs(ilaplace((randn(1)/s)*(H_w_n/H_w_d)))));     % gust along body z-axis

    % ------------------------------------------------------------------------
    % Constant gust value
    u_wg = 0.001;
    v_wg = 0.001;
    w_wg = 0.001;


    % Rotation matrix (right handed)
    R_roll = [...
                1, 0, 0;...
                0, cos(phi), sin(phi);...
                0, -sin(phi), cos(phi)];
    R_pitch = [...
                 cos(theta), 0, -sin(theta);...
                 0, 1, 0;...
                 sin(theta), 0, cos(theta)];
    R_yaw = [...
                 cos(psi), sin(psi), 0;...
                 -sin(psi), cos(psi), 0;...
                 0, 0, 1];
    R = R_roll*R_pitch*R_yaw;   % inertial to body


    % compute wind data in NED
    w_i = [w_ns; w_es; w_ds] + R' * [u_wg; v_wg; w_wg];
    w_n = w_i(1);
    w_e = w_i(2);
    w_d = w_i(3);
    
    % compute airspeed
    w_b = R*[w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg];
    u_r = u - w_b(1);
    v_r = v - w_b(2);
    w_r = w - w_b(3);

    Va = norm([u_r v_r w_r]);              % Airspeed
    alpha  = atan(w_r / u_r);              % Angle of attack
    beta = asin(v_r/norm([u_r v_r w_r]));  % Side-slip angle

    out = [Va; alpha; beta; w_n; w_e; w_d];

end

