function animate(data, dt)
% ANIMATE_LINE Animate a 3D line segment over time.
%
% data: 6 x N matrix
%       data(1:3, i) = [x1; y1; z1]
%       data(4:6, i) = [x2; y2; z2]
% dt:   time between frames (seconds)

N = size(data, 2);

% Extract full coordinate ranges
mins = min(data, [], 2);
maxs = max(data, [], 2);
pad = 0.3 * max(maxs - mins);

% ---- Create a TRUE 3D axes ----
fig = figure(99);
ax = axes('Parent', fig);
grid(ax, 'on')
axis(ax, 'equal')
view(ax, 3)                      % forces 3D view
hold(ax, 'on')

xlabel(ax, 'X')
ylabel(ax, 'Y')
zlabel(ax, 'Z')

xlim(ax, [mins(1)-pad, maxs(1)+pad])
ylim(ax, [mins(2)-pad, maxs(2)+pad])
zlim(ax, [mins(3)-pad-0.5, maxs(3)+pad])

% ---- Initial line ----
p1 = data(1:3, 1);
p2 = data(7:9, 1);

h = plot3(ax, ...
          [p1(1) p2(1)], ...
          [p1(2) p2(2)], ...
          [p1(3) p2(3)], ...
          'o-', 'LineWidth', 2);

% ---- Animation loop ----
for i = 2:N
    p1 = data(1:3, i);
    p2 = data(7:9, i);

    set(h, 'XData', [p1(1) p2(1)], ...
           'YData', [p1(2) p2(2)], ...
           'ZData', [p1(3) p2(3)]);

    drawnow;
    pause(dt);
end

end
