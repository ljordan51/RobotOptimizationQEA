function orientation = recalc_orientation(yaw, G, starting_orientation, pitch_adjustment)
        pitch = atan2(G(1), sqrt(G(2).^2 + G(3).^2)) - pitch_adjustment;
        roll = atan2(G(2), G(3));
        
        R_yaw = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        R_pitch = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        R_roll = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
        
        orientation = R_roll * R_pitch * R_yaw * starting_orientation;
end