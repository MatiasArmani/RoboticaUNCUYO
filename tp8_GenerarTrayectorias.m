% Cargar el modelo y configuración inicial
clc; clear; close all;
run("cargarModelo.m");
robot2;

% Vector de booleanos de sistemas de referencia a visualizar
sistemas = [1, 1, 1, 1, 1, 1, 1];  % (base + 6 articulaciones)

% Configurar el espacio de trabajo
workspace = [-1 1 -1 1 -1 1];  % Definir el espacio de trabajo

% Ploteo inicial del robot y la interfaz teach
% fig = figure;
R.plot(q);
R.teach();

% Guardar el objeto `R` en la estructura de datos de la interfaz
data.R = R; % Guardar el objeto R en 'data'
data.matrices = {}; % Inicializar lista de matrices capturadas
data.vectores_q = {}; % Inicializar lista de vectores q
guidata(fig, data); % Guardar la estructura 'data' en la GUI

% Crear un botón para capturar puntos
uicontrol('Style', 'pushbutton', 'String', 'Capturar Punto', ...
    'Position', [20, 20, 100, 30], ...
    'Callback', @capturarPunto);

% Graficar sistemas de referencia
hold on;
T = R.base.T;
handles = cell(1, length(sistemas)-1);
handles{1} = trplot(T, 'frame', sprintf('%d', 0), 'length', 0.8);
for i = 1:length(sistemas)-1
    if sistemas(i)
        handles{i} = trplot(T, 'frame', sprintf('%d', i), 'length', 0.8);  % Almacenar los handles
    end
end

% Bucle para actualizar los sistemas de referencia con cada cambio en las articulaciones
while ishandle(fig)
    % Obtener las posiciones articulares actuales desde los sliders
    q = R.getpos();
    
    % Reiniciar transformación homogénea base
    T = R.base.T;
    
    % Actualizar cada sistema de referencia
    for i = 1:length(sistemas)-1
        T_i = trotz(q(i) + R.offset(i)) * transl(dh(i,3), 0, dh(i,2)) * trotx(dh(i,4));
        T = T * T_i;
        if sistemas(i)
            set(handles{i}, 'Matrix', T);
        end
    end
    pause(0.01);
end

% Definir la función de callback
function capturarPunto(src, event)
    % Obtener los datos de la GUI
    fig = ancestor(src, 'figure');
    data = guidata(fig);
    R = data.R;
    
    % Obtener la configuración actual del robot
    q = R.getpos();
    T = R.fkine(q);
    
    % Agregar la matriz T y el vector q a la lista
    data.matrices{end+1} = T.T;
    data.vectores_q{end+1} = [q 1];
    
    % Actualizar los datos en la GUI
    guidata(fig, data);
    
    % Guardar en archivo
    fileID = fopen('anexos/matrices_transformacion_matiasTest.txt', 'a');
    % Primero escribir el vector q
    fprintf(fileID, '# Vector q:\n');
    fprintf(fileID, '%f %f %f %f %f %f\n', q);
    % Luego escribir la matriz T
    fprintf(fileID, '# Matriz T:\n');
    fprintf(fileID, '%f %f %f %f\n', T.T');
    fclose(fileID);
    
    disp('Punto capturado');
end
