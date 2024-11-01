% Cargar el modelo de la vitrina desde el archivo STL
modelo = stlread('VitrinaRobotica.stl');

% Parámetros de transformación para el modelo STL
traslacion = [1.5, 1.5, -1.6];  % Traslación en [x, y, z]
rotacion = [pi/2, -pi/2, 0];  % Rotación en radianes alrededor de [x, y, z]
escala = 0.0015;  % Factor de escala para reducir el tamaño del modelo

% Escalar el modelo STL
modelo_escalado = modelo.Points * escala;

% Aplicar la rotación al modelo STL
rotMatrix = makehgtform('xrotate', rotacion(1), 'yrotate', rotacion(2), 'zrotate', rotacion(3));
modelo_rotado = (rotMatrix(1:3, 1:3) * modelo_escalado')';

% Aplicar la traslación al modelo STL
modelo_transformado = modelo_rotado + traslacion;

% Configuración del gráfico 3D
fig = figure;
hold on;

% Graficar el modelo STL transformado
trisurf(modelo.ConnectivityList, modelo_transformado(:,1), modelo_transformado(:,2), modelo_transformado(:,3), ...
    'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5); % Color gris semitransparente
axis equal;
lighting gouraud;
camlight;