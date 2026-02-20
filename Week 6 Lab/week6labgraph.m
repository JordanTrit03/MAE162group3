data = readmatrix('week6labdata.txt');
x = data(:,4);   % 4th column
y = data(:,5);   % 5th column
figure;
plot(x, y, 'o-', 'LineWidth', 1.5);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Rover Trajectory');
grid on;
axis equal;
