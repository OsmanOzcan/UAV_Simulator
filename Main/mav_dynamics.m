

classdef mav_dynamics < handle

    properties
        ts_simulation
        state
        true_state
    end


    methods

        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts;

            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];

            addpath('../message_types');
            self.true_state = msg_state();
        end


        function self=update_state(self, forces_moments, MAV)
            %
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV.
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + (self.ts_simulation/2)*k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + (self.ts_simulation/2)*k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + (self.ts_simulation/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            % normalize the quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            self.update_true_state(forces_moments);
        end


        function xdot = derivatives(self, state, forces_moments, MAV)
            pn    = state(1);
            pe    = state(2);
            h     = -state(3);
            u     = state(4);
            v     = state(5);
            w     = state(6);
            e0    = state(7);
            e1    = state(8);
            e2    = state(9);
            e3    = state(10);
            p     = state(11);
            q     = state(12);
            r     = state(13);

            fx    = forces_moments(1);
            fy    = forces_moments(2);
            fz    = forces_moments(3);
            l     = forces_moments(4);
            m     = forces_moments(5);
            n     = forces_moments(6);
            
            mass   = MAV.mass;
            gamma1 = MAV.Gamma1;
            gamma2 = MAV.Gamma2;
            gamma3 = MAV.Gamma3;
            gamma4 = MAV.Gamma4;
            gamma5 = MAV.Gamma5;
            gamma6 = MAV.Gamma6;
            gamma7 = MAV.Gamma7;
            gamma8 = MAV.Gamma8;
            Jy     = MAV.Jy;
            
            % position kinematics
            pn_dot = (e1^2 + e0^2 - e2^2 - e3^2)*u + 2*(e1*e2 - e3*e0)*v + 2*(e1*e3 + e2*e0)*w;
            pe_dot = 2*(e1*e2 + e3*e0)*u + (e2^2 + e0^2 - e1^2 - e3^2)*v + 2*(e2*e3 - e1*e0)*w;
            pd_dot = 2*(e1*e3 - e2*e0)*u + 2*(e2*e3 + e1*e0)*v + (e3^2 + e0^2 - e1^2 - e2^2)*w;

            % position dynamics
            u_dot = r*v - q*w + fx/mass;
            v_dot = p*w - r*u + fy/mass;
            w_dot = q*u - p*v + fz/mass;
            
            % rotational kinematics
            e0_dot = (-p*e1 - q*e2 - r*e3)/2;
            e1_dot = (p*e0 + r*e2 - q*e3)/2;
            e2_dot = (q*e0 - r*e1 + p*e3)/2;
            e3_dot = (r*e0 + q*e1 - p*e2)/2;
            
            % rotational dynamics
            p_dot = gamma1*p*q - gamma2*q*r + gamma3*l + gamma4*n;
            q_dot = gamma5*p*r - gamma6*(p*p-r*r) + m/Jy;
            r_dot = gamma7*p*q - gamma1*q*r + gamma4*l + gamma8*n;
        
            % collect all the derivaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];

        end


        function self=update_true_state(self, forces_moments)

            eulerAngles = Quaternion2Euler(self.state(7:10));


            self.true_state.pn = self.state(1);          % pn
            self.true_state.pe = self.state(2);          % pe
            self.true_state.h = -self.state(3);          % pd
            self.true_state.phi = eulerAngles(1);        % phi
            self.true_state.theta = eulerAngles(2);      % theta
            self.true_state.psi = eulerAngles(3);        % psi
            self.true_state.p = self.state(11);          % p
            self.true_state.q = self.state(12);          % q
            self.true_state.r = self.state(13);          % r
            
            self.true_state.Vg = norm(self.state(4:6));  % Vg            
            self.true_state.Va = forces_moments(7);      % Va
            self.true_state.alpha = forces_moments(8);   % Angle of attack
            self.true_state.beta = forces_moments(9);    % Side-slip angle
            self.true_state.wn = forces_moments(10);     % w_n
            self.true_state.we = forces_moments(11);     % w_e
            self.true_state.wd = forces_moments(12);     % w_d
            self.true_state.chi = atan2(forces_moments(7)*sin(eulerAngles(3)) + forces_moments(11), forces_moments(7)*cos(eulerAngles(3)) + forces_moments(10));
        end
    end
end