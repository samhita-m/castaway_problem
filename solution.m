clc
clear all
close all

center = [0, 0];
radius = 1;
inner_radius = 0.225;

ac = 2*pi*rand;
r = (rand);
xc = (radius*r)*cos(ac) + center(1, 1);
yc = (radius*r)*sin(ac) + center(1, 2);

as = 2*pi*rand;
xs = (radius)*cos(ac) + center(1, 1);
ys = (radius)*sin(ac) + center(1, 2);

x_pose_shark = [xs];
y_pose_shark = [ys];
x_pose_castaway = [xc];
y_pose_castaway = [yc];

dist_btw_castandshark = norm([xs-xc, ys, yc]);

if dist_btw_castandshark == 0
    disp('Castaway cannot escape now :(')
else
    disp('Finding a path for the castaay to escape :)')
end

th = 0:180/50:360;
xso = radius* cosd(th) + center(1,1);
yso = radius* sind(th) + center(1,2);
xsi = inner_radius* cosd(th) + center(1,1);
ysi = inner_radius* sind(th) + center(1,2); 

count = 0;
d = 0.1;
theta_shark = 0.4*360/(2*pi); 
theta = rad2deg(ac);

dist_center_castaway = [xc - center(1, 1), yc - center(1, 2)];
dist_center_castaway_norm = norm(dist_center_castaway);

% making the castaway come to the center first
if dist_center_castaway_norm ~= 0
    num = dist_center_castaway_norm/0.1;
   if num <= 1
       %updating castaway's pose
       xc_current = center(1, 1);
       yc_current = center(1, 2);
       x_pose_castaway = vertcat(x_pose_castaway, xc_current);
       y_pose_castaway = vertcat(y_pose_castaway, yc_current);
       %updating shark's pose
       xa = -xs;
       ya = -ys;
       xb = xs;
       yb = ys;
       xp = xc;
       yp = yc;
       [cw] = find_direction(xa, ya, xb, yb, xp, yp);
       if cw == 1
           theta = theta + theta_shark;
       else
           theta = theta - theta_shark;
       end
       xs_current = radius*cosd(theta) + center(1, 1);
       ys_current = radius*sind(theta) + center(1, 2);
       x_pose_shark = vertcat(x_pose_shark, xs_current);
       y_pose_shark = vertcat(y_pose_shark, ys_current);
   else 
       loop_num = floor(num);
       for j = 1:loop_num
           pause(0.5);
           %updating castaway's pose
           x1 = x_pose_castaway(j, 1);
           y1 = y_pose_castaway(j, 1);
           x0 = center(1, 1);
           y0 = center(1, 2);
           v = [x1-x0, y1-y0];
           v_norm = norm(v);
           u = v/v_norm;           
           xc_current = x1 - d*u(1, 1);
           yc_current = y1 - d*u(1, 2);
           x_pose_castaway = vertcat(x_pose_castaway, xc_current);
           y_pose_castaway = vertcat(y_pose_castaway, yc_current);
           
           %updating shark's pose
           xa = -x_pose_shark(j,1);
           ya = -y_pose_shark(j,1);
           xb = x_pose_shark(j,1);
           yb = y_pose_shark(j,1);
           xp = x_pose_castaway(j, 1);
           yp = y_pose_castaway(j, 1);
           [cw] = find_direction(xa, ya, xb, yb, xp, yp);
           if cw == 1
               theta = theta + theta_shark;
           else
               theta = theta - theta_shark;
           end
           xs_current = radius*cosd(theta) + center(1, 1);
           ys_current = radius*sind(theta) + center(1, 2);
           x_pose_shark = vertcat(x_pose_shark, xs_current);
           y_pose_shark = vertcat(y_pose_shark, ys_current);
       end
       xc_current = center(1, 1);
       yc_current = center(1, 2);
       x_pose_castaway = vertcat(x_pose_castaway, xc_current);
       y_pose_castaway = vertcat(y_pose_castaway, yc_current);
       
       %updating shark's pose
       xa = -x_pose_shark(j+1,1);
       ya = -y_pose_shark(j+1,1);
       xb = x_pose_shark(j+1,1);
       yb = y_pose_shark(j+1,1);
       xp = x_pose_castaway(j+1, 1);
       yp = y_pose_castaway(j+1, 1);
       [cw] = find_direction(xa, ya, xb, yb, xp, yp);
       if cw == 1
           theta = theta + theta_shark;
       else
           theta = theta - theta_shark;
       end
       xs_current = radius*cosd(theta) + center(1, 1);
       ys_current = radius*sind(theta) + center(1, 2);
       x_pose_shark = vertcat(x_pose_shark, xs_current);
       y_pose_shark = vertcat(y_pose_shark, ys_current);
   end
end

[rc, ~] = size(x_pose_castaway);
for i = rc:rc+19
    x0 = center(1, 1);
    y0 = center(1, 2);
    x1 = x_pose_shark(i, 1);
    y1 = y_pose_shark(i, 1);
    if i == 1
        v = [x1-x0, y1-y0];
        v_norm = norm(v);
        u = v/v_norm;
        castaway_pose = [x0, y0] - d*u;
        xc_current = castaway_pose(1, 1);
        yc_current = castaway_pose(1, 2);
    else
        v = [x1-x0, y1-y0];
        v_norm = norm(v);
        u = v/v_norm;
        opptoshark = [x0, y0] - 0.225*u;
        dis = [x_pose_castaway(i, 1)- opptoshark(1, 1), y_pose_castaway(i, 1)- opptoshark(1, 2)];
        dist_inner = norm(dis);
        dd = [x_pose_castaway(i, 1), y_pose_castaway(i, 1)];
        dist_inner_to_center = norm(dd);
%         if (dist_inner < 0.1)|| dist_inner_to_center > 1
%             disp('ok')
%             i-rc
%         else 
%             disp('still finding')
%         end
        px = x_pose_castaway(i, 1);
        py = y_pose_castaway(i, 1);
        ax = x_pose_shark(i, 1);
        ay = y_pose_shark(i, 1);
        bx = -x_pose_shark(i, 1);
        by = -y_pose_shark(i, 1);
        ap = [px-ax, py-ay];
        ab = [bx-ax, by-ay];
        ab_norm = norm(ab)^2;
        ap_ab_dot = dot(ap, ab);
        t = ap_ab_dot/ab_norm;
        x0 = ax + (bx-ax)*t;
        y0 = ay + (by-ay)*t;

        v = [x1-x0, y1-y0];
        v_norm = norm(v);
        u = v/v_norm;
        castaway_pose = [x0, y0] - d*u;
        xc_current = castaway_pose(1, 1);
        yc_current = castaway_pose(1, 2);
    end
    xa = -x_pose_shark(i,1);
    ya = -y_pose_shark(i,1);
    xb = x_pose_shark(i,1);
    yb = y_pose_shark(i,1);
    xp = x_pose_castaway(i, 1);
    yp = y_pose_castaway(i, 1);
    [cw] = find_direction(xa, ya, xb, yb, xp, yp);
    if cw == 1
        theta = theta + theta_shark;
    else
        theta = theta - theta_shark;
    end
    xs_current = radius*cosd(theta) + center(1, 1);
    ys_current = radius*sind(theta) + center(1, 2);

    x_pose_castaway = vertcat(x_pose_castaway, xc_current);
    y_pose_castaway = vertcat(y_pose_castaway, yc_current);

    x_pose_shark = vertcat(x_pose_shark, xs_current);
    y_pose_shark = vertcat(y_pose_shark, ys_current);

  
end


% Plotting everything here
[num_iter, ~] = size(x_pose_castaway);
figure(1);

m = moviein(50);

for kk = 1:num_iter
    pause(0.5)
    clf(figure(1))
    xc = x_pose_castaway(kk, 1);
    yc = y_pose_castaway(kk, 1);
    xs = x_pose_shark(kk, 1);
    ys = y_pose_shark(kk, 1);
    f = plot(xc, yc, '*');
    hold on
    f1 = plot(xs, ys, 'o');
    f2 = plot(xso, yso, 'k');
    f3 = plot(center(1, 1), center(1, 2), '+');
    xlim([-1.5 1.5])
    ylim([-1.5 1.5])
    title('Castaway Escapes?')
    xlabel('x units')
    ylabel('y units')
    F = getframe(figure(1));
    m(:, kk) = F; 
end
movie(gcf, m);



