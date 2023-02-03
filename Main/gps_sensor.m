classdef gps_sensor < handle

    properties
        time
        gps_rate
        time_last_gps
        gps_initialized
        nou_n
        nou_e
        nou_d
        y_gps_n
        y_gps_e
        y_gps_h
        y_gps_Vg
        y_gps_course
    end

    methods

        function self = gps_sensor
            self.time = 0;
            self.gps_rate = 1;
            self.time_last_gps = self.time;
            self.gps_initialized = 0;
            self.nou_n = 0;
            self.nou_e = 0;
            self.nou_d = 0;
            self.y_gps_n = 0;
            self.y_gps_e = 0;
            self.y_gps_h = 0;
            self.y_gps_Vg = 0;
            self.y_gps_course = 0;
        end



        function y = update(self, uu, SENSOR, Ts)
        
            Va      = uu(1);
            alpha   = uu(2);
            beta    = uu(3);
            wn      = uu(4);
            we      = uu(5);
            wd      = uu(6);
            pn      = uu(7);
            pe      = uu(8);
            pd      = uu(9);
            u       = uu(10);
            v       = uu(11);
            w       = uu(12);
            e0      = uu(13);
            e1      = uu(14);
            e2      = uu(15);
            e3      = uu(16);
            p       = uu(17);
            q       = uu(18);
            r       = uu(19);
            t       = uu(20);
            
            eulerAngles = Quaternion2Euler([e0 e1 e2 e3]);
            phi      = eulerAngles(1);       % roll angle         
            theta    = eulerAngles(2);       % pitch angle     
            psi      = eulerAngles(3);       % yaw angle
            
            
            if self.gps_initialized == 0

                self.y_gps_n = pn;
                self.y_gps_e = pe;
                self.y_gps_h = -pd;
            
                self.y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
                self.y_gps_course = atan2((Va*sin(psi)+we), (Va*cos(psi)+wn));
                
                self.gps_initialized = 1;

            else
                if self.time >= self.time_last_gps + self.gps_rate
            
                    self.nou_n = exp(-SENSOR.gps_beta*self.gps_rate)*self.nou_n + SENSOR.gps_n_sigma*randn(1);
                    self.nou_e = exp(-SENSOR.gps_beta*self.gps_rate)*self.nou_e + SENSOR.gps_e_sigma*randn(1);
                    self.nou_d = exp(-SENSOR.gps_beta*self.gps_rate)*self.nou_d + SENSOR.gps_h_sigma*randn(1);
            
                    self.y_gps_n = pn + self.nou_n;
                    self.y_gps_e = pe + self.nou_e;
                    self.y_gps_h = -pd + self.nou_d;
                
                    self.y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2) + SENSOR.gps_Vg_sigma*randn(1);
                    self.y_gps_course = atan2((Va*sin(psi)+we), (Va*cos(psi)+wn)) + SENSOR.gps_course_sigma*randn(1);
                    

                    self.y_gps_n = pn;
                    self.y_gps_e = pe;
                    self.y_gps_h = -pd;
                    self.y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
                    self.y_gps_course = atan2((Va*sin(psi)+we), (Va*cos(psi)+wn));

                    self.time_last_gps = self.time;
                end
            end
            
            self.time = self.time + Ts;
            
            y = [...
                self.y_gps_n;...
                self.y_gps_e;...
                self.y_gps_h;...
                self.y_gps_Vg;...
                self.y_gps_course;...
                ];
            
        end
    end
end


