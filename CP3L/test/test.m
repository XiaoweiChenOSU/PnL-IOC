clear all;
% t1 = (-2:0.01:0);
% y1 = exp(2*t1);
% 
% t2 = (0:0.01:2);
% y2 = exp(-2*t2);
% 
% plot(t1,y1,t2,y2);

t2 = (-2:0.01:2);
y2 = 4./(4+t2.^2);

plot(t2,y2);


        if abs(cT(2)-cH) > 3
            continue;
        end