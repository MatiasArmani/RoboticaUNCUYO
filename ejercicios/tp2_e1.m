clc, clear, close

I = eye(4);
Ta = [  0.5 -0.866 0
      0.866    0.5 0
          0      0 1];
Tb = [0 0 1
    -1 0 0
    0 -1 0];
Tc = [0.5 -0.750 -0.433
    0.866 0.433 0.250
    0 -0.5 0.866];
figure(1);
view(30,30);
trplot(I,'frame','0');
hold on
trplot(Ta,'frame','a','color','red');
title('Consigna A');
hold off
figure(2);
view(30,30);
trplot(I,'frame','0');
hold on
trplot(Tb,'frame','b','color','red');
title('Consigna B');
hold off
figure(3);
view(30,30);
trplot(I,'frame','0');
hold on
trplot(Tc,'frame','c','color','red');
title('Consigna C');
hold off