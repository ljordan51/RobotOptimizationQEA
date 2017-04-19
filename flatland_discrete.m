% Initialize position vector at initial position of (4,0)
% Inititialize heading
% Define lambda = 1/16

% Compute gradient function

% while grad > 0.01:
    % Evaluate gradient vector
    % Turn in direction of gradient vector
    % Drive forward lambda*del
    % Store new position

% Set up ROS stuff
% pub = rospublisher('/raw_vel');
% msg = rosmessage(pub);

% Initialize variables
r = [4; 1];
d = 10.025*0.0254; % wheelbase (m)
heading = pi;
lambda = 1/16;
delta = 1.2;
omega = 0.5; % angular velocity for rotating

% Calculate wheel speeds to use when rotating
V_L_rotating = -omega*d/2;
V_R_rotating = omega*d/2;
V_linear = 0.2;

syms t x y grad_f
f = x.*y - x.^2 - y.^2 -2*x - 2*y + 4;
grad_f(x,y) = [diff(f, x); diff(f, y)];

while norm(grad_f(r(1),r(2))) > 0.01
    grad = grad_f(r(1),r(2));
    
    target_heading = pi - atan(grad(2)./grad(1));
    
    delta_heading = target_heading - heading;
    fprintf('Target heading: %.2f degrees\tNeed to turn %.2f degrees\n', double(target_heading)*180/pi, double(delta_heading) * 180 / pi);
    
    time = double(delta_heading / omega);
    
    direction = sign(time)
    
    time = abs(time)
    
    % Start rotating the Neato
%     msg.Data = direction*[V_L_rotating V_R_rotating];
%     send(pub, msg);
    
    % Wait for it to turn
    pause(time);
    
    % Stop rotating
%     msg.Data = [0 0];
%     send(pub, msg);
    
    % Get magnitude of gradient in meters
    mag = lambda * norm(grad) * 0.3048;
    
    % Calculate how long we need to move forward
    time = double(mag / V_linear);
    
    % Go forward for time
%     msg.Data = [V_linear V_linear];
%     send(pub, msg);
    
    % Wait for it to move
    pause(time);
    
    % Stop moving
%     msg.Data = [0 0];
%     send(pub, msg);
    
    r = double(r + lambda.*grad)
    heading = target_heading;
    
    lambda = lambda * delta;
end
