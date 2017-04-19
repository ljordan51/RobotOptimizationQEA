function [orientation, pos] = resetFrame()
    orientation = [cosd(45) -sind(45) 0;...
               sind(45) cosd(45) 0;
               0 0 1];
    pos = [0; 1; 0];
end