function plot_path_from_encoder_data(left, right, time)
    % Initialize stuff
    d = 0.248; % wheelbase (m)
    % Orient the Neato with its left wheel at the origin and facing up the y-
    % axis
    heading_init = pi/2;
    center_x_init = 0;
    r_neato_init = [center_x_init; 0]; % position of center of Neato
    
    % Calculate velocites (in m/s) from positions for each point in time
    delta_t = diff(time);
    v_L = diff(left)./delta_t;
    v_R = diff(right)./delta_t;

    % Calculate the angular velocity about the ICC for each point in time
    omega = (v_R-v_L)/d;

    % Calculate R (distance from the center of the robot to the ICC)
    R_over_time = v_L./omega + d/2;
    % Fix any NaNs
%     ave_R = mean(R_over_time(~isnan(R_over_time)));
    R_over_time(isnan(R_over_time)) = 100;
    
    % Calculate rotation given dt for each point in time
    delta_heading_over_time = delta_t.*omega;

    % Make a matrix to keep track of the wheel positions (like r_neato_init)
    % at all the points in time. Also create a vector to store the heading
    % vs time.
    r_neato_over_time = zeros(2,length(time));
    r_neato_over_time(:,1) = r_neato_init;
    heading_over_time = [heading_init; zeros(length(time)-1,1)];

    figure(3); clf; hold on;
    title('The Neato as She Goes');
    % Loop through all the points in time to rotate the Neato relative to the
    % ICC
    
    for i = 2:length(time)-1
        % Pull out the old heading, change in heading and Neato position for time i
        heading_old = heading_over_time(i-1);
        delta_heading = delta_heading_over_time(i-1);
        r_neato_rel_origin_old = r_neato_over_time(:,i-1);
        fprintf('Timestep %4.0f\tHeading: %2.1f degrees\tR: %0.5f\n', i, heading_old/pi*180, R_over_time(i-1));
        
        % Get ICC position
        r_ICC_rel_origin = get_r_ICC_rel_origin(r_neato_rel_origin_old, R_over_time(i-1), heading_old);
        % Shift Neato position so that the ICC is at the origin
        r_neato_rel_ICC_old = r_neato_rel_origin_old - r_ICC_rel_origin;

        % Create the rotation matrix
        rot_mat = [cos(delta_heading), -sin(delta_heading); sin(delta_heading), cos(delta_heading)];

        % Rotate the Neato about the origin
        r_neato_rel_ICC_new = rot_mat * r_neato_rel_ICC_old;
        
        % Shift the Neato back
        r_neato_rel_origin_new = r_neato_rel_ICC_new + r_ICC_rel_origin;
        
        % Update the "over time" variables
        heading_over_time(i) = heading_old + delta_heading;
        r_neato_over_time(:,i) = r_neato_rel_origin_new;
        
        % Plot stuff
        plot(r_neato_rel_origin_new(1), r_neato_rel_origin_new(2), 'rx');
%         plot(r_ICC_rel_origin(1), r_ICC_rel_origin(2), 'ko');

        % Assignment requirements:
        % Calculate delta_x and delta_y
        delta_r = r_neato_rel_origin_new - r_neato_rel_origin_old;
        pause(0.1);
    end
    
    function r_ICC_rel_origin = get_r_ICC_rel_origin(neato_center_pos, R, heading)
%         r_c = mean(neato_center_pos, 2); % wheelbase center position
        t = heading-pi/2; % angle between line through wheelbase and x-axis
        r_ICC_rel_origin = neato_center_pos - [R*cos(t); R*sin(t)];
    end

end