plot(y_p,x_p);
hold on
title('Prueba para hallar ICRr');
xlabel('Desplazamiento en x');
ylabel('Desplazamiento en y');
maxx=max(x_p);
minx=min(x_p);
maxy=max(y_p);
miny=min(y_p);
promx=(maxx-minx)/2;
promy=(maxy-miny)/2;
x_gra=promx+minx
y_gra=promy+miny
plot(y_gra,x_gra,'b--o');
legend('Dsplazamiento del robot','Punro ICRr')
grid();