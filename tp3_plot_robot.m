clc; clear; close all;

robot2;

% Vector de posiciones articulares que se desea analizar
q = [pi/4, pi/6, -pi/4, pi/3, pi/6, 0.05];

% Vector de booleanos de sistemas de referencia a visualizar
sistemas = [1, 1, 1, 1, 1, 1, 1];  % (base + 6 articulaciones)

% Ploteo del robot en la posición definida en b
workspace = [-1 1 -1 1 -1 1];  % Definir el espacio de trabajo
R.plot(q, 'workspace', workspace, 'scale', 0.5, 'jointdiam', 0.3, 'notiles');

% Graficar sistemas de referencia
hold on;
for i = 0:length(sistemas)-1
    if sistemas(i+1)
        trplot(R.A(1:i, q), 'frame', num2str(i), 'color', 'b', 'length', 0.1);
    end
end
hold off;