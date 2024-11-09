% clc;
clear; close all;

robot2;
q = [33.0, 45.6, -17.7, -69.7, 48.6, 19.0]*pi/180;

% Vector de booleanos de sistemas de referencia a visualizar
sistemas = [1,1,1,1,1,1];
% Ploteo del robot en la posición definida en b
workspace = [-1 1 -1 1 -1 1];  % Definir el espacio de trabajo

fig = figure;
R.plot(q);
R.teach(q);


% Graficar sistemas de referencia
hold on;


T = R.base.T;
% Graficar los sistemas de referencia iniciales
handles = cell(1, length(sistemas)-1);
handles{1} = trplot(T, 'frame', sprintf('%d', 0), 'length', 0.8);
for i = 1:length(sistemas)-1
    if sistemas(i)
        handles{i} = trplot(T, 'frame', sprintf('%d', i), 'length', 0.8);  % Almacenar los handles
    end
end

% Bucle para actualizar los sistemas de referencia con cada cambio en las articulaciones
while true

    if ~ishandle(fig)
        disp('Figura cerrada. Terminando ejecución.');
        break;  % Salir del bucle si la figura se cierra
    end

    % Obtener las posiciones articulares actuales desde los sliders
    q = R.getpos();
    
    % Reiniciar transformación homogénea base
    T = R.base.T;
    
    % Actualizar cada sistema de referencia
    for i = 1:length(sistemas)-1
        % Calcular transformación homogénea de la articulación i
        T_i = trotz(q(i)+R.offset(i)) * transl(dh(i,3),0,dh(i,2)) * trotx(dh(i,4));
        
        % Actualizar transformación homogénea global
        T = T * T_i;

        % Verificar si se debe actualizar este sistema de referencia
        if sistemas(i)
            % Actualizar la matriz de transformación del objeto gráfico (sin volver a graficar)
            set(handles{i}, 'Matrix', T);
        end
    end
    
    % Pausa para permitir interacción con los sliders
    pause(0.01);
end
