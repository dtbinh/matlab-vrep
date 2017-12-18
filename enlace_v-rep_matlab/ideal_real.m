xxx=x0:0.01:x1;
yyy=y0:0.01:y1;
plot(yyy,xxx);
hold on
plot(y_p,x_p);
title('Evolución de la posición');
xlabel('Distancia en x');
ylabel('Distancia en y')
legend('Recorrido Ideal','Recorrido Real')
grid