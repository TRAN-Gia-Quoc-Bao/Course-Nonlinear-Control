function xp = robotArm(x)
global J Bm k m l g
u = x(5);
xp(1) = x(2);
xp(2) = -1/J*(k*(x(1)-x(3)) + m*g*l*cos(x(1)));
xp(3) = x(4);
xp(4) = 1/J*(-Bm*x(4) + k*(x(1)-x(3)) + u);
end

