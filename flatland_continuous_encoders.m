function flatland_continuous_encoders
    % Set up ROS stuff
    pub = rospublisher('/raw_vel');
    enc_sub = rossubscriber('/encoders');
    speed_message = rosmessage(pub);

    % Initialize variables
    d = 10.025/12; % wheelbase (ft)
    a = 0.05;
    r_neato_init = [4; 0];
    heading_init = pi/2;

    % Predicted path stuff
    syms t x(t) y(t) T u v
    f(u,v) = a*(u.*v - u.^2 - v.^2 -2*u - 2*v + 4);
    grad = [diff(f,u) diff(f,v)];
    ode_x = diff(x) == a*(y-2*x-2);
    ode_y = diff(y) == a*(x-2*y-2);
    cond_x = x(0) == 4;
    cond_y = y(0) == 1;
    [r_x(t), r_y(t)] = dsolve([ode_x; ode_y],[cond_x; cond_y]);


    T = [diff(r_x,t) diff(r_y,t) 0];
    T_hat = T/norm(T);
    N = diff(T_hat);
    % angular velocity is omega, which is velocity/R
    omega = cross(T_hat,N);
    R = norm(diff([r_x, r_y]))./omega(3);
    V_L = omega(3).*(R - d/2);
    V_R = omega(3).*(R + d/2);

    % Keep track of the Neato's position and heading vs time (just for plotting).
    r_neato_actual_over_time = [r_neato_init];
    heading_over_time = [heading_init];


    timer = tic;
    t = 0;
    times = [t];
    past_wheel_odoms = [0; 0]; % Keep track of the last wheel odometry

    while true
        t = toc(timer);
        delta_t = t - times(end);
        
        % Read positions from encoders
        new_wheel_odom = enc_sub.LatestMessage.Data;
        delta_odom = new_wheel_odom - past_wheel_odoms; % format: [delta_L delta_R}
        
        % Calculate predicted velocity and position
        vl_pred = double(subs(V_L));
        vr_pred = double(subs(V_R));
        r_pred = [r_x(t); r_y(t)];
        
        % Calculate actual velocity and position from encoders
        v_L = diff([past_wheel_odoms(1,end) new_wheel_odom(1)])/delta_t;
        v_R = diff([past_wheel_odoms(2,end) new_wheel_odom(2)])/delta_t;
        omega = (v_R-v_L)/d;
        % Handle infinite R's
        if (v_L - v_R) < 0.0001
            R = 100;
        else
            R = v_L./omega + d/2;
        end
        
        delta_heading = delta_t*omega;
        heading = heading_over_time(end) + delta_heading;
        fprintf('Timestep %4.0f\tHeading: %2.1f degrees\tR: %0.5f\n', length(times)+1, heading/pi*180, R);

        % Get ICC position
        r_ICC_rel_origin = r_neato_actual_over_time(:,end) - [R*cos(heading-pi/2); R*sin(heading-pi/2)];
        % Shift Neato position so that the ICC is at the origin
        r_neato_rel_ICC_old = r_neato - r_ICC_rel_origin;
        % Create the rotation matrix
        rot_mat = [cos(delta_heading), -sin(delta_heading); sin(delta_heading), cos(delta_heading)];
        % Rotate the Neato about the origin
        r_neato_rel_ICC_new = rot_mat * r_neato_rel_ICC_old;
        % Shift the Neato back
        r_neato_rel_origin_new = r_neato_rel_ICC_new + r_ICC_rel_origin;

        % Update the "over time" variables - really just to use for
        % plotting
        heading_over_time(end+1) = {heading_old + delta_heading};
        r_neato_actual_over_time(end+1) = {r_neato_rel_origin_new};
        times(end+1) = t;
        
        if norm(grad(r_pred(1), r_pred(2))) < 0.18
            break
        end
        speed_message.Data = [vl_pred(3) vr_pred(3)];
        send(pub, speed_message);
    end


    speed_message.Data = [0 0];
    send(pub, speed_message);
    
    r_pred = [r_x(times); r_y(times)];
    plot(r_pred(1,:), r_pred(2,:), 'r');
    plot(r_neato_actual_over_time(1,:), r_neato_actual_over_time(2,:), 'b');
end