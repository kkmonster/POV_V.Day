clc
clear all



for i=0:200
    xx(i+1) =    -sind(i*1.8);
    yy(i+1) =    -cosd(i*1.8);
    
%     for y =15 : 16
     y=15;
    yp(1+i) = (20.0 + (4.0+y)*yy(i+1));
    xp(1+i) = (20.0 + (4.0+y)*xx(i+1));
    
    index(1+i) = int16(yp(1+i))*43 + int16(xp(1+i));
%     end
end
index