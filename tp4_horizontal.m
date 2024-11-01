clc; clear; close all;

% Cargar el robot
robot2;

% Definir parámetros de la cuadrícula
x = linspace(-1.2, 1.2, 50); % Rango X en metros
y = linspace(-1.2, 1.2, 50); % Rango Y en metros
z = 0.15; % Altura fija para el análisis

% Crear matrices para almacenar resultados
[X, Y] = meshgrid(x, y);
soluciones_validas = zeros(size(X));

% Analizar cada punto
for i = 1:length(x)
    for j = 1:length(y)
        % Crear matriz de transformación para el punto actual
        T = transl(X(i,j), Y(i,j), z) * rpy2tr(0, 0, 0);
        
        % Obtener todas las soluciones
        try
            q_todas = cinInversa(R,T,0,dh,[0.1,0.1,0.1,0.1,0.1,0.1]);
            validas = 0;
            
            % Verificar cada solución
            for k = 1:8
                q = q_todas(k,:);
                if all(~isnan(q))
                    dentro_limites = true;
                    for m = 4
                        if q(m) < R.qlim(m,1) || q(m) > R.qlim(m,2)
                            dentro_limites = false;
                            break;
                        end
                    end
                    if dentro_limites
                        validas = validas + 1;
                    end
                end
            end
            soluciones_validas(i,j) = validas;
        catch
            continue;
        end
    end
end

% Visualizar resultados
figure;
surf(X, Y, soluciones_validas/8, 'EdgeColor', 'none'); % Dividir por 8 para normalizar
colormap(jet(8));
colorbar('Ticks', (0:7)/8, 'TickLabels', 0:7); % Ajustar colorbar para mostrar valores reales
title('Espacio de Trabajo - Número de Soluciones Válidas');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Proporción de soluciones válidas');
axis equal;
grid on;

% Dibujar el robot en posición home
hold on;
R.plot([0 0 0 0 0 0], 'nobase', 'noshadow');