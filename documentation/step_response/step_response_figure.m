% https://uk.mathworks.com/campaigns/products/control-tutorials.html#
% DC Motor Speed: System Analysis 

V = 4.5;
J = 0.000000807;
b = 0.000000414;
K = 0.0059;
R = 1.72;
L = 0.000106;
s = tf('s');
P_motor = V*K/((J*s+b)*(L*s+R)+K^2);
figure(1);
step(P_motor)
displaySettleTime(P_motor,0.02)
