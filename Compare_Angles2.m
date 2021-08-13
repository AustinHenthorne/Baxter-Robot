function [Compare2] = Compare_Angles2(Joint_Angles)
% Austin Henthorne MEC529 Final Project
% This function takes in the joint angles 
% And outputs if it is in the range of the robot or not
% STEP 6

%Input: Compare_Angles([1; 1; 1; 1; 1; 1; 1])
Theta_1 = Joint_Angles(1,1);
Theta_2 = Joint_Angles(2,1);
Theta_3 = Joint_Angles(3,1);
Theta_4 = Joint_Angles(4,1);
Theta_5 = Joint_Angles(5,1);
Theta_6 = Joint_Angles(6,1);
Theta_7 = Joint_Angles(7,1);

if Theta_1 > 1.047 || Theta_1 < -2.147
    a = 1;
else
    a = 2;
end

if Theta_2 > 2.618 || Theta_2 < -0.052
    b = 1;
else 
     b = 2;
end

if Theta_3 > 2.094 || Theta_3 < -1.571
    c = 1;
else
    c = 2;
end

if Theta_4 > 3.028 || Theta_4 < -3.028
    d = 1; 
else
      d = 2;
end

if Theta_5 > 0.890 || Theta_5 < -2.461
    e = 1;
else
    e = 2;
end

if Theta_6 > 3.059 || Theta_6 < -3.059
    f = 1;
else
    f = 2;
end

if Theta_7 > 3.059 || Theta_7 < -3.059
    g = 1;
else
    g = 2;
end

if a+b+c+d+e+f+g == 7
    A = 'The configuration is not reachable for the second hand';
    Compare2 = A;
else
    B = 'The configuration is reachable for the second hand';
    Compare2 = B;

end
