robot2;

% Definir la matriz de transformación homogénea T
T = [
   -0.2409    0.2486    0.9382    1.1250
   -0.0618   -0.9686    0.2408    0.1605
    0.9686    0.0000    0.2487    0.1873
         0         0         0    1.0000
    ]; % Ejemplo de matriz T

% Inicializar la lista para almacenar los resultados de q
resultados_q = zeros(50, size(q0, 2));

% Generar 50 valores aleatorios de q0
for i = 1:50
    q0 = 2 * pi * (rand(1, size(q0, 2)) - 0.5); % Valores aleatorios entre -pi y pi

    % Ejecutar la función de cinemática inversa (supongamos que se llama cinInversa)
    q = cinInversa(R,T,1,dh,q0);
    
    % Almacenar el resultado en la lista de q
    resultados_q(i, :) = q;
end

% Mostrar el listado de q
disp(resultados_q);