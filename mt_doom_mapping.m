clearvars;
% Set up ROS stuff
sub = rossubscriber('/accel');
enc = rossubscriber('/encoders');
bump = rossubscriber('/bump');
pub = rospublisher('/raw_vel');
speed_message = rosmessage(pub);

% Initialize variables
d = 10.025*0.0254; % wheelbase (m)
r = d/2;
omega_max = 2;
max_speed = 1;
yaw_correction = 2; % scalar to try to correct yaw
rotation_speed_adjustment = 0.5;

G = [0 0 0];

room_axes = eye(3);

r_over_time = [0; 0; 0]; % position of Neato over time, relative to origin

pitch_adjustment = 0.33; % Pitch of Neato on flat ground due to jacked chassis

yaw = pi/2;
enc_0 = receive(enc);
enc_0 = enc_0.Data;
pause(0.1);


enc_start = receive(enc);
mode = 'nothing';
dist_since_going_straight = 0;
enc_vals_starting_rotations = enc_start.Data;

should_continue = 1; % Set to 0 in debugger to stop and plot

while should_continue
    % Stop if bumper is hit
    if sum(bump.LatestMessage.Data) > 0
        break
    end
    
    G = receive(sub);
    G = G.Data;
    
    pitch = atan2(G(1), sqrt(G(2).^2 + G(3).^2)) - pitch_adjustment;
    roll = atan2(G(2), G(3));
    R_yaw = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R_pitch = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
    R_roll = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    orientation = R_roll * R_pitch * R_yaw * room_axes;
    
    enc_start = receive(enc);
    enc_start = enc_start.Data;
    
    if abs(G(2)) < 0.005
        % Update robot's orientation after rotating
        if strcmp(mode, 'rotating')
            dist = enc_start - enc_vals_starting_rotations;
            mean_dist = mean(abs(dist))*sign(dist(2));
            delta_yaw = mean_dist*2/d;
            yaw = yaw + delta_yaw;
            z_prime = orientation(:,3);
            R = vrrotvec2mat([z_prime' delta_yaw]);
            orientation = R*orientation;
            enc_vals_starting_rotations = 0;
            was_just_rotating = 0;
            enc_vals_before_going_forward = enc_start;
        end
        
        % Go forward
        speed_scalar = abs(pitch/pi*4)/2;
        speed = speed_scalar*max_speed;
        % Make sure our speed is within range but will actually move the
        % robot
        if speed > 0.33
            speed = 0.33;
        elseif speed < 0.1
            speed = 0.1;
        end
        % Move
        speed_message.Data = [speed speed];
        send(pub, speed_message);
        pause(1);
        % Stop
        speed_message.Data = [0 0];
        send(pub, speed_message);
        mode = 'going_straight';
        
        % Debugging
        enc_now = receive(enc);
        dist = enc_now.Data - enc_start;
        fprintf('Went forward %.3f m\n', dist);
        
    else % Rotate
        % Update Neato's position after moving forward
        if strcmp(mode, 'going_straight')
            dist = enc_start - enc_vals_before_going_forward;
            r_change_vector = dist(1)*orientation(:,1);
            r_over_time(:,end+1) = r_over_time(:,end) + r_change_vector;

            pause(0.3); % Let it stop before reading the accelerometer

            % Update robot coordinate system
            G = receive(sub);
            G = G.Data;
            yaw = atan2(orientation(2,1), orientation(1,1));
            orientation = recalc_orientation(yaw, G, room_axes, pitch_adjustment);
            
            enc_vals_starting_rotations = enc_start;
        end
        
        % Now actually rotate the Neato
        omega = -omega_max*G(2); % Scale based on magnitude of G_y
        if abs(omega) < 0.1 % Don't turn slower than 0.2 s^-1
            omega = 0.1*sign(omega);
        end
        speed_message.Data = omega*d*[1 -1]*rotation_speed_adjustment;
        send(pub, speed_message);
        pause(0.5/rotation_speed_adjustment);
        speed_message.Data = [0 0];
        send(pub, speed_message);
        
        enc_now = receive(enc);
        enc_now = enc_now.Data;
        dist = enc_now - enc_start;
        ave_dist = mean(abs(dist))*sign(dist(2));
        mode = 'rotating';
%         dist_since_rotating = dist_since_rotating + ave_dist;
        
        % Debugging
        dist = enc_now - enc_vals_starting_rotations;
        mean_dist = mean(abs(dist))*sign(dist(2));
        delta_yaw = mean_dist*2/d;
        yaw_now = yaw + delta_yaw;
        delta_since_start = enc_now - enc_0;
        fprintf('Total yaw: %.3f deg\tDist: %.5f, %.5f\tTotal: %.5f, %.5f\n', yaw_now*180/pi,dist(1), dist(2), delta_since_start(1), delta_since_start(2));
        pause(0.2);
        was_just_rotating = 1;
    end
    
    
end


speed_message.Data = [0 0];
send(pub, speed_message);

figure(2); clf; hold on; axis equal;
plot3(0.76,1.5,0.23,'rx'); % End point
plot3(r_over_time(1,:), r_over_time(2,:), r_over_time(3,:)); % Neato
plot3([-0.23,0.92,0.92,-0.23,-0.23],[-0.3,-0.3,1.52,1.52,-0.3],zeros(1,5),'k'); % Mount Doom(TM)