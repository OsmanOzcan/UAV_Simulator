

function out = forces_moments(state, delta, WIND, MAV, sim_time)

    % relabel the inputs
    pn      = state(1);
    pe      = state(2);
    pd      = state(3);
    u       = state(4);
    v       = state(5);
    w       = state(6);
    p       = state(11);
    q       = state(12);
    r       = state(13);


    eulerAngles = Quaternion2Euler(state(7:10));
    phi      = eulerAngles(1);       % roll angle         
    theta    = eulerAngles(2);       % pitch angle     
    psi      = eulerAngles(3);       % yaw angle 


    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    
    out = wind_simulation(state, WIND, sim_time);
    Va = out(1);
    alpha = out(2);
    beta = out(3);
    w_n = out(4); 
    w_e = out(5); 
    w_d = out(6);

    
    % aerodynamic coefficients
    sigma = (1+exp(-MAV.M*(alpha-MAV.alpha0))+exp(MAV.M*(alpha+MAV.alpha0)))/...
            ((1+exp(-MAV.M*(alpha-MAV.alpha0)))*(1+exp(MAV.M*(alpha+MAV.alpha0))));
    Cl = (1-sigma)*(MAV.C_L_0+MAV.C_L_alpha*alpha) + ...
          sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    Cd = MAV.C_D_0 + ((MAV.C_L_0+MAV.C_L_alpha*alpha)^2)/(pi*MAV.e*MAV.b^2/MAV.S_wing);
    Cx = -Cd*cos(alpha)+Cl*sin(alpha);
    Cx_q = -MAV.C_D_q*cos(alpha)+MAV.C_L_q*sin(alpha);
    Cx_delta_e = -MAV.C_D_delta_e*cos(alpha)+MAV.C_L_delta_e*sin(alpha);
    Cz = -Cd*sin(alpha)-Cl*cos(alpha);
    Cz_q = -MAV.C_D_q*sin(alpha)-MAV.C_L_q*cos(alpha);
    Cz_delta_e = -MAV.C_D_delta_e*sin(alpha)-MAV.C_L_delta_e*cos(alpha);


    
    % compute external forces and torques on aircraft
    Force(1) =  -MAV.mass*MAV.gravity*sin(theta) + ...
                0.5*MAV.rho*(Va^2)*MAV.S_wing * (Cx + Cx_q*(MAV.c/(2*Va))*q + Cx_delta_e*delta_e) + ...
                0.5*MAV.rho*MAV.S_prop*MAV.C_prop * ((MAV.k_motor*delta_t)^2-Va^2);

    Force(2) =  MAV.mass*MAV.gravity*cos(theta)*sin(phi) + ...
                0.5*MAV.rho*(Va^2)*MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta*beta + MAV.C_Y_p*(MAV.b/(2*Va))*p + ...
                MAV.C_Y_r*(MAV.b/(2*Va))*r + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);

    Force(3) =  MAV.mass*MAV.gravity*cos(theta)*cos(phi) + ...
                0.5*MAV.rho*(Va^2)*MAV.S_wing * (Cz + Cz_q*(MAV.c/(2*Va))*q + Cz_delta_e*delta_e);
    

    Torque(1) =  0.5*MAV.rho*(Va^2)*MAV.S_wing * MAV.b * ...
       (MAV.C_ell_0 + MAV.C_ell_beta*beta + MAV.C_ell_p*(MAV.b/(2*Va))*p + MAV.C_ell_r*(MAV.b/(2*Va))*r +...
        MAV.C_ell_delta_a*delta_a + MAV.C_ell_delta_r*delta_r) - ...
        MAV.k_T_P*((MAV.k_Omega*delta_t)^2);

    Torque(2) = 0.5*MAV.rho*(Va^2)*MAV.S_wing * MAV.c * ...
        (MAV.C_m_0 + MAV.C_m_alpha*alpha + MAV.C_m_q*(MAV.c/(2*Va))*q + MAV.C_m_delta_e*delta_e);

    Torque(3) = 0.5*MAV.rho*(Va^2)*MAV.S_wing * MAV.b * ...
        (MAV.C_n_0 + MAV.C_n_beta*beta + MAV.C_n_p*(MAV.b/(2*Va))*p + MAV.C_n_r*(MAV.b/(2*Va))*r + ...
         MAV.C_n_delta_a*delta_a + MAV.C_n_delta_r*delta_r);
   
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



