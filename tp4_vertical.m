clc; clear; close all;

% Cargar el robot
robot2;

% Definir parámetros de la cuadrícula
z = linspace(-1.2, 1.2, 200); % Rango Z en metros
x = linspace(-1.2, 1.2, 200); % Rango X en metros
y_fijo = 0.15; % Valor fijo de Y

% Crear matrices para almacenar resultados
[X, Z] = meshgrid(x, z);
soluciones_validas = zeros(size(X));

% Manejar el pool de trabajadores
pool = gcp('nocreate'); % Verificar si ya existe un pool
if isempty(pool)
    parpool; % Crear un pool si no existe
end

% Analizar cada punto en paralelo
parfor idx = 1:numel(X)
    % Crear matriz de transformación para el punto actual en el plano ZX con Y fijo
    T = transl(X(idx), y_fijo, Z(idx)) * rpy2tr(0, 0, 0);
    
    % Obtener todas las soluciones
    try
        q_todas = cinInversa(R, T, 0, dh, [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);
        validas = 0;
        
        % Verificar cada solución
        for k = 1:8
            q = q_todas(k,:);
            if all(~isnan(q))
                dentro_limites = true;
                for m = 1:6
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
        soluciones_validas(idx) = validas;
    catch
        continue;
    end
end

% Cerrar el pool de trabajadores si se ha creado
delete(gcp('nocreate'));

% Visualizar resultados
figure;
surf(X, Z, soluciones_validas/8, 'EdgeColor', 'none'); % Dividir por 8 para normalizar
colormap(jet(8));
colorbar('Ticks', (0:7)/8, 'TickLabels', 0:7); % Ajustar colorbar para mostrar valores reales
title('Espacio de Trabajo en el plano ZX - Número de Soluciones Válidas');
xlabel('X (m)');
ylabel('Z (m)');
zlabel('Proporción de soluciones válidas');
axis equal;
grid on;