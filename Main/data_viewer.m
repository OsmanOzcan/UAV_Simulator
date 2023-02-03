classdef data_viewer < handle
    
    properties
    	pn_handle
    	pe_handle
    	h_handle
        wn_handle
    	Va_handle
    	alpha_handle
    	beta_handle
        we_handle
    	phi_handle
    	theta_handle
        psi_handle
        Vg_handle
    	chi_handle
    	p_handle
    	q_handle
    	r_handle
        de_handle
        da_handle
        dr_handle
        dt_handle
        bx_handle
        by_handle
        bz_handle
        map_handle
        plot_initialized
        time
        plot_rate
        time_last_plot
    end


    methods

        function self = data_viewer
            self.pn_handle = [];
            self.pe_handle = [];
            self.h_handle = [];
            self.wn_handle = [];
            self.Va_handle = [];
            self.alpha_handle = [];
            self.beta_handle = [];
            self.we_handle = [];
            self.phi_handle = [];
            self.theta_handle = [];
            self.psi_handle = [];
            self.Vg_handle = [];
            self.chi_handle = [];
            self.p_handle = [];
            self.q_handle = [];
            self.r_handle = [];
            self.de_handle = [];
            self.da_handle = [];
            self.dr_handle = [];
            self.dt_handle = [];
            self.bx_handle = [];
            self.by_handle = [];
            self.bz_handle = [];
            self.map_handle = [];
            self.time = 0;
            self.plot_rate = 0.1;
            self.time_last_plot = self.time;
            self.plot_initialized = 0;    
        end

        
        function self=update(self, true, estimated, commanded, Ts, axisMatrix)
            
            if self.plot_initialized == 0
                %clf(axisMatrix, "reset");

                hold(axisMatrix(1,1), "on");
                self.pn_handle = self.graph_y_yhat(self.time, axisMatrix(1,1), true.pn, estimated(1), [], 'North Distance');
                    
                hold(axisMatrix(1,2), "on");
                self.Va_handle = self.graph_y_yhat_yd(self.time, axisMatrix(1,2), true.Va, estimated(4), commanded(4), [], 'Airspeed (Va)');

                hold(axisMatrix(1,3), "on");
                self.phi_handle = self.graph_y_yhat_yd(self.time, axisMatrix(1,3), true.phi/pi*180, estimated(7)/pi*180, commanded(7)/pi*180, [], 'Phi Angle (Φ)');

                hold(axisMatrix(1,4), "on");
                self.p_handle = self.graph_y_yhat(self.time, axisMatrix(1,4), true.p/pi*180, estimated(10)/pi*180, [], 'Phi Rate');

                hold(axisMatrix(2,1), "on");
                self.pe_handle = self.graph_y_yhat(self.time, axisMatrix(2,1), true.pe, estimated(2), [], "East Distance");

                hold(axisMatrix(2,2), "on");
                self.alpha_handle = self.graph_y_yhat(self.time, axisMatrix(2,2), true.alpha/pi*180, estimated(5)/pi*180, [], 'Angle of Attack (\alpha)');

                hold(axisMatrix(2,3), "on");
                self.theta_handle = self.graph_y_yhat_yd(self.time, axisMatrix(2,3), true.theta/pi*180, estimated(8)/pi*180, commanded(8)/pi*180, [], 'Theta Angle (θ)');

                hold(axisMatrix(2,4), "on");
                self.q_handle = self.graph_y_yhat(self.time, axisMatrix(2,4), true.q/pi*180, estimated(11)/pi*180, [], 'Theta Rate');

                hold(axisMatrix(3,1), "on");
                self.h_handle = self.graph_y_yhat_yd(self.time, axisMatrix(3,1), true.h, estimated(3), commanded(3), [], 'Altitude');

                hold(axisMatrix(3,2), "on");
                self.beta_handle = self.graph_y_yhat(self.time, axisMatrix(3,2), true.beta/pi*180, estimated(6)/pi*180, [], 'Side-Slip Angle (\beta)');

                hold(axisMatrix(3,3), "on");
                self.psi_handle = self.graph_y_yhat(self.time, axisMatrix(3,3), true.psi/pi*180, true.psi/pi*180, [], 'Psi Angle (Ψ)');

                hold(axisMatrix(3,4), "on");
                self.r_handle = self.graph_y_yhat(self.time, axisMatrix(3,4), true.r/pi*180, estimated(12)/pi*180, [], 'Psi Rate');

                hold(axisMatrix(4,1), "on");
                self.wn_handle = self.graph_y(self.time, axisMatrix(4,1), norm([true.wn true.we true.wd]), [], 'Wind Speed');

                hold(axisMatrix(4,2), "on");
                self.Vg_handle = self.graph_y_yhat(self.time, axisMatrix(4,2), true.Vg, estimated(13), [], 'Ground Speed');

                hold(axisMatrix(4,3), "on");
                self.chi_handle = self.graph_y_yhat_yd(self.time, axisMatrix(4,3), true.chi/pi*180, estimated(9)/pi*180, commanded(9)/pi*180, [], '\chi');

                hold(axisMatrix(4,4), "on");
                %self.bx_handle = self.graph_y_yhat(self.time, axisMatrix(4,4), true.bx, true.bx, [], 'Bias');
                %self.by_handle = self.graph_y_yhat(self.time, axisMatrix(4,4), true.by, true.by, [], []);
                %self.bz_handle = self.graph_y_yhat(self.time, axisMatrix(4,4), true.bz, true.bz, [], []);
                
                hold(axisMatrix(5,1), "on");
                self.de_handle = self.graph_y(self.time, axisMatrix(5,1), commanded(13)/pi*180, [], "Elevator");

                hold(axisMatrix(5,2), "on");
                self.da_handle = self.graph_y(self.time, axisMatrix(5,2), commanded(14)/pi*180, [], "Aileron");

                hold(axisMatrix(5,3), "on");
                self.dr_handle = self.graph_y(self.time, axisMatrix(5,3), commanded(15)/pi*180, [], "Rudder");

                hold(axisMatrix(5,4), "on");
                self.dt_handle = self.graph_y(self.time, axisMatrix(5,4), commanded(15)*100, [], "Thrust (%)");
                
                hold(axisMatrix(6,1), "on");
                self.map_handle = self.graph_3D_map(true.pe, axisMatrix(6,1), true.pn, true.h, [], "3D Map");

                self.plot_initialized = 1;
            else
                if self.time >= self.time_last_plot + self.plot_rate

                    self.graph_y_yhat(self.time, axisMatrix(1,1), true.pn, estimated(1), self.pn_handle);
                    self.graph_y_yhat(self.time, axisMatrix(2,1), true.pe, estimated(2), self.pe_handle);
                    self.graph_y_yhat_yd(self.time, axisMatrix(3,1), true.h, estimated(3), commanded(3), self.h_handle);
                    self.graph_y(self.time, axisMatrix(4,1), norm([true.wn true.we true.wd]), self.wn_handle);
                    self.graph_y_yhat_yd(self.time, axisMatrix(1,2), true.Va, estimated(4), commanded(4), self.Va_handle);
                    self.graph_y_yhat(self.time, axisMatrix(2,2), true.alpha/pi*180, estimated(5)/pi*180, self.alpha_handle);
                    self.graph_y_yhat(self.time, axisMatrix(3,2), true.beta/pi*180, estimated(6)/pi*180, self.beta_handle);
                    self.graph_y_yhat(self.time, axisMatrix(4,2), true.Vg, estimated(13), self.Vg_handle);
                    self.graph_y_yhat_yd(self.time, axisMatrix(1,3), true.phi/pi*180, estimated(7)/pi*180, commanded(7)/pi*180, self.phi_handle);
                    self.graph_y_yhat_yd(self.time, axisMatrix(2,3), true.theta/pi*180, estimated(8)/pi*180, commanded(8)/pi*180, self.theta_handle);
                    self.graph_y_yhat(self.time, axisMatrix(3,3), true.psi/pi*180, true.psi/pi*180, self.psi_handle);
                    self.graph_y_yhat_yd(self.time, axisMatrix(4,3), true.chi/pi*180, estimated(9)/pi*180, commanded(9)/pi*180, self.chi_handle);
                    self.graph_y_yhat(self.time, axisMatrix(1,4), true.p/pi*180, estimated(10)/pi*180, self.p_handle);
                    self.graph_y_yhat(self.time, axisMatrix(2,4), true.q/pi*180, estimated(11)/pi*180, self.q_handle);
                    self.graph_y_yhat(self.time, axisMatrix(3,4), true.r/pi*180, estimated(12)/pi*180, self.r_handle);
%                     self.graph_y_yhat(self.time, axisMatrix(4,4), true.bx, true.bx, self.bx_handle);
%                     self.graph_y_yhat(self.time, axisMatrix(4,4), true.by, true.by, self.by_handle);
%                     self.graph_y_yhat(self.time, axisMatrix(4,4), true.bz, true.bz, self.bz_handle);
                    self.graph_y(self.time, axisMatrix(5,1), commanded(13)/pi*180, self.de_handle);
                    self.graph_y(self.time, axisMatrix(5,2), commanded(14)/pi*180, self.da_handle);
                    self.graph_y(self.time, axisMatrix(5,3), commanded(15)/pi*180, self.dr_handle);
                    self.graph_y(self.time, axisMatrix(5,4), commanded(16)*100, self.dt_handle);
                    self.graph_3D_map(true.pe, axisMatrix(6,1), true.pn, true.h, self.map_handle);
                    self.time_last_plot = self.time;
                end
                self.time = self.time + Ts;
            end
        end


        function handle = graph_y_yhat_yd(self, t, axisLabel, y, yhat, yd, handle, lab)
            if isempty(handle)
                handle(1)   = plot(axisLabel, t, y, "b");
                handle(2)   = plot(axisLabel, t, yhat, "g--");
                handle(3)   = plot(axisLabel, t, yd, "r-.");
                title(axisLabel, "\color{white}" + lab);
            else
                l = findall(axisLabel, "type", "line"); % fig is the handle of figure or uifigure object
                l(1).XData;
                l(2).XData;
                l(3).XData;
                xlim(axisLabel, [0, t]);

                l(3).XData = [l(3).XData, t];
                l(3).YData = [l(3).YData, y];
                l(2).XData = [l(2).XData, t];
                l(2).YData = [l(2).YData, yhat];
                l(1).XData = [l(1).XData, t];
                l(1).YData = [l(1).YData, yd];
            end
        end


        function handle = graph_y_yhat(self, t, axisLabel, y, yhat, handle, lab)
            if isempty(handle)
                handle(1)   = plot(axisLabel, t, y,'b');
                handle(2)   = plot(axisLabel, t, yhat,'g--');
                title(axisLabel, "\color{white}" + lab);
            else
                l = findall(axisLabel, "type", "line"); % fig is the handle of figure or uifigure object
                l(1).XData;
                l(2).XData;
                xlim(axisLabel, [0, t]);

                l(2).XData = [l(2).XData, t];
                l(2).YData = [l(2).YData, y];
                l(1).XData = [l(1).XData, t];
                l(1).YData = [l(1).YData, yhat];
            end
        end
        
        
        function handle = graph_y(self, t, axisLabel, y, handle, lab)
  
            if isempty(handle)
                handle    = plot(axisLabel, t, y,'b');
                title(axisLabel, "\color{white}" + lab);
            else
                l = findall(axisLabel, "type", "line"); % fig is the handle of figure or uifigure object
                l(1).XData;
                xlim(axisLabel, [0, t]);

                l(1).XData = [l(1).XData, t];
                l(1).YData = [l(1).YData, y];
            end
        end

        function handle = graph_3D_map(self, x, axisLabel, y, z, handle, lab)
  
            if isempty(handle)
                handle    = plot3(axisLabel, x, y, z,'b');
                title(axisLabel, "\color{white}" + lab);
                view(axisLabel, 32,47);
                grid(axisLabel, "on");
            else
                l = findall(axisLabel, "type", "line"); % fig is the handle of figure or uifigure object
                l(1).XData;

                l(1).XData = [l(1).XData, x];
                l(1).YData = [l(1).YData, y];
                l(1).ZData = [l(1).ZData, z];
            end
        end
    end
end