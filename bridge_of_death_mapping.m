syms t
% d = 0.25;
d = 10.025*0.0254; % wheelbase (m)
s = 5;
a = 0.4;
l = 0.4;
r = [-2*a.*((l - cos(t/s)).*cos(t/s) + (1 - l)),  2*a*(l - cos(t/s)).*sin(t/s), 0];
% r = -2*a*[-cos(t/s)*(1 - cos(t/s)) sin(t/s)*(1 - cos(t/s)) 0];
% Take the derivative to get the tangential velocity vector
T = diff(r);
T_hat = T/norm(T);
N = diff(T_hat);
% angular velocity is omega, which is velocity/R
w = cross(T_hat,N);
R = norm(diff(r))./w(3);
Vl = w(3).*(R - d/2);
Vr = w(3).*(R + d/2);

pub = rospublisher('/raw_vel');
speed_message = rosmessage(pub);
enc = rossubscriber('/encoders');
bump = rossubscriber('/bump');

wheel_positions = [0; 0];
times = 0;

enc_pos = receive(enc);
old_enc_pos = enc_pos.Data;
heading = 0;

timer = tic;
t = 0;
while toc(timer) < 16
    vl = double(subs(Vl));
    vr = double(subs(Vr));
    speed_message.Data = [vl vr];
    send(pub, speed_message);
    pause(0.1);
    t = toc(timer);
    delta_t = t - times(end);
    enc_pos = receive(enc);
    enc_pos = enc_pos.Data;
%     dist = enc_pos - old_enc_pos;
%     dist_center = mean(dist);
    wheel_positions(:,end+1) = enc_pos;
    % Stop if we've hit a bumper
    if (sum(bump.LatestMessage.Data) > 0)
       speed_message.Data = [0, 0];
       send(pub, speed_message);
       break;
    end
    times(end+1) = t;
end

speed_message.Data = [0 0];
send(pub, speed_message);

% wheel_positions = wheel_positions - old_enc_pos;

plot_path_from_encoder_data(wheel_positions(1,:), wheel_positions(2,:), times);

% Go forward for delta_t
% Calc change in encoder position
% Calculate ICC position
% Calculate change in central angle
% Rotate robot position vector about ICC
