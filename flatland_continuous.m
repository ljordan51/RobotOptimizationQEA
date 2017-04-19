clearvars;
% Set up ROS stuff
pub = rospublisher('/raw_vel');
speed_message = rosmessage(pub);

% Initialize variables
d = 10.025/12; % wheelbase (ft)
a = 0.05;

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
w = cross(T_hat,N);
R = norm(diff([r_x, r_y]))./w(3);
V_L = w(3).*(R - d/2);
V_R = w(3).*(R + d/2);

timer = tic;

t = 0;

while true
    t = toc(timer);
    vl = double(subs(V_L));
    vr = double(subs(V_R));
    r = [r_x(t) r_y(t)];
    if norm(grad(r(1), r(2))) < 0.18
        break
    end
    speed_message.Data = [vl(3) vr(3)];
    send(pub, speed_message);
end


speed_message.Data = [0 0];
send(pub, speed_message);