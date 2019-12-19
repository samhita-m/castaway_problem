function [cw] = find_direction(xa, ya, xb, yb, xp, yp)
%If clockewise cw = 1
%If clockewise cw = 0
%If on the line cw = 1

%a is the shark's opp side
%b is the shark's pose
%p is the castaway's pose

ab = [xb-xa, yb-ya];
ap = [xp-xa, yp-ya];


cross_product = ab(1, 1)*ap(1, 2) - ab(1, 2)*ap(1, 1);
if cross_product >= 0
    cw = 1;
else
    cw = 0;
end
