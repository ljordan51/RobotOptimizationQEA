function visualizeAxes(orientation, pos, newFigure)
    if nargin < 3
        newFigure = true;
    end
    if newFigure
        figure;
    end
    view(3);
    mArrow3(pos, pos + orientation(:,1), 'color', 'b', 'tipWidth', .05, 'stemWidth',.01);
    hold on;
    mArrow3(pos, pos + orientation(:,2), 'color', 'k', 'tipWidth', .05, 'stemWidth',.01);
    mArrow3(pos, pos + orientation(:,3), 'color', 'r', 'tipWidth', .05, 'stemWidth',.01);
    legend({'x','y','z'});
    xlim([-1 3]);
    ylim([-1 3]);
    zlim([-1 3]);
    daspect([1 1 1]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
end