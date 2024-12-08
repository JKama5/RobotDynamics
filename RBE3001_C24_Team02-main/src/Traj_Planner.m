%Trajectory planner
classdef Traj_Planner
    properties
    end

    methods
        %constructor
        function self=Traj_Planner()

        end
        
        %solves for a cubic (3rd order) polynomial trajectory between two via-points
        %t0: initial time (in seconds)
        %tf: final time (in seconds)
        %v0: initial velocity (degrees/s or mm/s)
        %vf: final velocity (degrees/s or mm/s)
        %p0: initial position (degrees or mm)
        %pf: final position (degrees or mm)
        %coefficents: 4x1 array of the coefficents of the polynomial
        function coefficents=cubic_traj(self,t0,tf,v0,vf,p0,pf)
            coefficents=zeros(4,1);
            constraints=[p0;v0;pf;vf];
            A= [1   t0  t0^2    t0^3
                0   1   2*t0    3*t0^2
                1   tf  tf^2    tf^3
                0   1   2*tf    3*tf^2];
            coefficents=A\constraints;
        end
        function coefficentsq = quintic_traj(self,t0,tf,v0,vf,a0,af,p0,pf)
            coefficentsq=zeros(6,1);
            constraints=[p0;v0;a0;pf;vf;af];
            A = [1   t0  t0^2  t0^3    t0^4     t0^5
                 0   1   2*t0  3*t0^2  4*t0^3   5*t0^4
                 0   0   2     6*t0    12*t0^2  20*t0^3
                 1   tf  tf^2  tf^3    tf^4     tf^5
                 0   1   2*tf  3*tf^2  4*tf^3   5*tf^4
                 0   0   2     6*tf    12*tf^2  20*tf^3];
            coefficentsq = A\constraints;
        end
        %runs cubic traj witht the corisponding inputs and plots the
        %position and velocity graph
        function plotCubicTraj(self,t0,tf,v0,vf,p0,pf)
            a=self.cubic_traj(t0,tf,v0,vf,p0,pf);
            t=[t0:(tf-t0)/100:tf];
            figure;
            plot(t,a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3)
            title('position over time - cubic')
            ylabel('position (degrees or mm)')
            xlabel('time (s)')
            figure;
            plot(t,a(2)+2*a(3)*t+3*a(4)*t.^2)
            title('velocity over time - cubic')
            ylabel('velocity (degrees/s or mm/s)')
            xlabel('time (s)')
            figure;
            plot(t,2*a(3)+6*a(4)*t)
            title('acceleration over time - cubic')
            ylabel('velocity (degrees/(s^2) or mm/(s^2)')
            xlabel('time (s)')
        end

        function plotCubicTrajWithCoefficents(self,a,t0,tf,typeofplot)
            t=[t0:(tf-t0)/100:tf];
            figure;
            if typeofplot>=1
                plot(t,a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3)
                title('position over time - cubic')
                ylabel('position (degrees or mm)')
                xlabel('time (s)')
            end
            if typeofplot>=2
                figure;
                plot(t,a(2)+2*a(3)*t+3*a(4)*t.^2)
                title('velocity over time - cubic')
                ylabel('velocity (degrees/s or mm/s)')
                xlabel('time (s)')
            end
            if typeofplot>=3
                figure;
                plot(t,2*a(3)+6*a(4)*t)
                title('acceleration over time - cubic')
                ylabel('velocity (degrees/(s^2) or mm/(s^2)')
                xlabel('time (s)')
            end
        end

        function plotqunticTraj(self,t0,tf,v0,vf,a0,af,p0,pf)
            a=self.quintic_traj(t0,tf,v0,vf,a0,af,p0,pf);
            t=[t0:(tf-t0)/100:tf];
            figure;
            plot(t,a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3+a(5)*t.^4+a(6)*t.^5)
            title('position over time - quintic')
            ylabel('position (degrees or mm)')
            xlabel('time (s)')
%             figure;
%             plot(t,a(2)+2*a(3)*t+3*a(4)*t.^2+4*a(5)*t.^3+5*a(6)*t.^4)
%             title('velocity over time - quintic')
%             ylabel('velocity (degrees/s or mm/s)')
%             xlabel('time (s)')
%             figure;
%             plot(t,2*a(3)+6*a(4)*t+12*a(5)*t.^2+20*a(6)*t.^3)
%             title('acceleration over time - quintic')
%             ylabel('velocity (degrees/(s^2) or mm/(s^2)')
%             xlabel('time (s)')
        end

        function plotQuinticTrajWithCoefficents(self,a,t0,tf,typeofplot)
            t=[t0:(tf-t0)/100:tf];
            figure;
            if typeofplot>=1
                plot(t,a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3)
                title('position over time - cubic')
                ylabel('position (degrees or mm)')
                xlabel('time (s)')
            end
            if typeofplot>=2
                figure;
                plot(t,a(2)+2*a(3)*t+3*a(4)*t.^2)
                title('velocity over time - cubic')
                ylabel('velocity (degrees/s or mm/s)')
                xlabel('time (s)')
            end
            if typeofplot>=3
                figure;
                plot(t,2*a(3)+6*a(4)*t)
                title('acceleration over time - cubic')
                ylabel('velocity (degrees/(s^2) or mm/(s^2)')
                xlabel('time (s)')
            end
        end
    end
end