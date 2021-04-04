clear all;
close all;

print_plot = 0;

if print_plot == 1
  graphics_toolkit("gnuplot");
else
  graphics_toolkit("qt");
endif

A = 3.9083*10^(-3);
B = -5.7750*10^(-7);

Rr = 0.1; %référence en série rajouté en kOhm
t=0:100;

R = 1+A*t+B*t.*t;

figure(1)
plot(t,R);
title ("R(t)");
xlabel("t (°C)");
ylabel("R(t)/R0");
if print_plot == 1
  print resitor_variation.fig
endif

a = B;
b = A;
c = 1-R;
delta2 = b*b - 4*a*c;
tm = (-b+sqrt(delta2))/2/a;

tm2 = (R-1)/A;

figure(2)
plot(t,tm,t,tm2);
title("t retrouvé");

figure(3)
plot(t,tm2-tm);
ylim([-1.5 0]);
title ("error for linear simplification");
xlabel("t (°C)");
ylabel("linear error");
if print_plot == 1
  print linear_error.fig
endif

Np = 1024;
Rr = linspace(0.1,10,1024);
delta = zeros(Np,1);
for k=1:Np
  Req=R+Rr(k);
  U = R./Req;
  delta(k) = U(end)-U(1);
endfor

%U=R./Req;
figure(4)
plot(Rr,delta);
title("variation de tension mesuré en fonction de la résistance de référence");
xlabel("R_r/R_0");
ylabel("\Delta V");
if print_plot == 1
  print maximum_relative_voltage_variation.fig
endif

Rr = 1;
Req = R+Rr;
U = R./Req;
figure(5)
plot(t,U);
title("tension relative aux borne du capteur");


m=floor(1024*U);

figure(6)
plot(t,m);
title("mesure arduino en fonction de la température");
xlabel("t (°C)");
ylabel("ADC output");
if print_plot == 1
  print arduino_mesure.fig
endif