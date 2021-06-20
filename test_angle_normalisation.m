x = linspace(-2*pi,2*pi);


y = mod(x + pi, 2*pi) - pi;


plot(x,y)
grid minor
ylim([-pi,pi])