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
function capturarPunto(~, ~)
    % Recuperar los datos de la GUI
    data = guidata(gcf);
    R = data.R; % Extraer el objeto R guardado
    
    % Obtener la posición actual del efector final
    T_efector = R.fkine(R.getpos());
    
    % Convertir el objeto SE3 a una matriz 4x4
    T_efector = T_efector.T;
    
    % Verificar que la matriz tenga dimensiones correctas y mostrarla
    disp('Matriz T_efector capturada:');
    disp(T_efector);
    
    % Almacenar la matriz de transformación completa
    data.matrices{end+1} = T_efector; % Agregar la matriz a la lista
    guidata(gcf, data); % Actualizar los datos de la GUI
    
    % Guardar la lista de matrices en un archivo al final de la ejecución
    saveMatricesToFile(data.matrices);
end

% Función para guardar las matrices en un archivo de texto
function saveMatricesToFile(matrices)
    fileID = fopen('matrices_transformacion3.txt', 'w');
    
    if fileID == -1
        error('No se pudo abrir el archivo para escribir.');
    end
    
    % Recorrer cada matriz y escribirla en el archivo
    for i = 1:length(matrices)
        T_efector = matrices{i}; % Extraer la matriz 4x4
        
        % Escribir cada elemento de la matriz en el archivo
        fprintf(fileID, '%.4f %.4f %.4f %.4f\n', T_efector(1, :));
        fprintf(fileID, '%.4f %.4f %.4f %.4f\n', T_efector(2, :));
        fprintf(fileID, '%.4f %.4f %.4f %.4f\n', T_efector(3, :));
        fprintf(fileID, '%.4f %.4f %.4f %.4f\n', T_efector(4, :));
        %fprintf(fileID, '\n'); % Línea en blanco entre matrices
    end
    
    fclose(fileID);
    disp('Las matrices han sido guardadas en matrices_transformacion.txt');
end
