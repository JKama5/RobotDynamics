classdef Task2functions
    % Plans trajectories
    
    properties
        height = 200; % millimeters

        waypoint = [185; 0; 240];
    end
    
    methods
        function self = Task2functions()
        end
        




        % Define addresses
        ADDR_OPERATING_MODE = 11;       % Operating Mode register
        ADDR_TORQUE_ENABLE = 64;        % Torque Enable register
        
        % Define values
        DXL_ID = 1;                     % Dynamixel ID
        PROTOCOL_VERSION = 2.0;         % Protocol version (adjust if necessary)
        NEW_MODE = 5;                   % New operating mode (e.g., 5 = Current-Based Control)
        
        % Disable Torque
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, 0);
        disp('Torque disabled.');
        
        % Set new operating mode
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, NEW_MODE);
        disp(['Operating mode set to ', num2str(NEW_MODE)]);
        
        % Re-enable Torque
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, 1);
        disp('Torque re-enabled.');




% Calculates an LSPB trajectory
        % Takes initial and final positions, times, and maximum velocity
        function [t, x, v] = lspb_traj(~, x0, xf, t0, tf, vmax)
            % Time and displacement calculation
            T = tf - t0;         % Total time
            delta_x = xf - x0;   % Total displacement
            
            % Calculate the blend time and linear velocity
            tb = (vmax * T - delta_x) / vmax; % Blend time
            if tb < 0
                error('cannot get here in this time');
            end
            tb = tb / 2;         % Blend duration (half for acceleration+ and deceleration-)
            vl = vmax;           % Linear velocity
            
            % Time instances
            t = linspace(t0, tf, 100); % Generate time vector
            
            % Position and velocity profiles
            x = zeros(size(t));
            v = zeros(size(t));
            
            for i = 1:length(t)
                if t(i) < t0 + tb
                    % Acceleration phase (parabolic)
                    x(i) = x0 + 0.5 * vl / tb * (t(i) - t0)^2;
                    v(i) = vl / tb * (t(i) - t0);
                elseif t(i) <= tf - tb
                    % Linear phase
                    x(i) = x0 + vl * (t(i) - t0 - tb / 2);
                    v(i) = vl;
                else
                    % Deceleration phase (parabolic)
                    x(i) = xf - 0.5 * vl / tb * (tf - t(i))^2;
                    v(i) = vl / tb * (tf - t(i));
                end
            end
        end
    end
end