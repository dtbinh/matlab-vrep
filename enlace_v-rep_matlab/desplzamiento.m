plot(y_p,x_p)
hold on
title('Evolución de la posición');
xlabel('Desplazamiento en x m');
ylabel('Desplazamiento en y m');
plot(y1,x1,'o')
legend('Desplazamiento del robot','Goal');
grid