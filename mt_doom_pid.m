clearvars;
% Set up ROS stuff
sub = rossubscriber('/accel');
bump = rossubscriber('/bump');
pub = rospublisher('/raw_vel');
speed_message = rosmessage(pub);

% Initialize variables
d = 10.025*0.0254; % wheelbase (m)
omega_max = 2;

G = [0 0 0];

pause(0.1);

while 1
    % Stop if bumper is hit
    if sum(bump.LatestMessage.Data) > 0
        break
    end
    
    pitch = atand(-G(1)/G(3));
    
    G = sub.LatestMessage.Data;
    
    if abs(G(2)) < 0.005
        % Go forward
        speed_message.Data = abs(pitch/45)*[0.3 0.3];
        send(pub, speed_message);
        pause(0.5);
        speed_message.Data = [0 0];
        send(pub, speed_message);
    else
        % Rotate
        omega = -omega_max*G(2); % Scale based on magnitude of G_y
        if abs(omega) < 0.1
            omega = 0.1*sign(omega);
        end
        speed_message.Data = omega*d*[1 -1];
        send(pub, speed_message);
        pause(0.5);
        speed_message.Data = [0 0];
        send(pub, speed_message);
    end
    
    pause(0.2);
    
end


speed_message.Data = [0 0];
send(pub, speed_message);