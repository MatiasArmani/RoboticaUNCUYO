clc; clear; close all;

robot2;
%cinematicaInversaIkine;

num_tests = 100; % Número de pruebas
max_norm = 0; % Inicializar la norma máxima
max_norm_matrix = []; % Inicializar la matriz con la mayor norma
count_large_norms = 0; % Inicializar el contador de normas mayores a 1e-4
failed_q_randoms = []; % Inicializar la lista para guardar los vectores q_random fallidos

tic; % Iniciar el temporizador

for test = 1:num_tests
    % Generar una configuración aleatoria del robot
    q_random = [30, 50, 70, 120, 20, 10]*pi/180; % Generar ángulos aleatorios en el rango [-pi, pi] radianes
    q_random2 = (rand(1, 6) * 2 - 1) * pi;

    % Calcular la matriz de transformación T mediante cinemática directa
    T_original = R.fkine(q_random2).double;

    % Calcular la cinemática inversa usando la función cinematicaInversaIk
   % q_solutions = cinematicaInversaIkine.cinematicaInversaIk(R, T_original, q_random2, false);
    q_solutions = cinInversa(R, T_original, 0, dh, zeros(1,6));
    q_solutions*180/pi

    % Inicializar la bandera para contar solo una vez por prueba
    large_norm_found = false;

    % Calcular las matrices T para las 8 soluciones devueltas
    for i = 1:size(q_solutions, 1)
        T_solution = R.fkine(q_solutions(i, :)).double;

        % Calcular la diferencia entre T_solution y T_original
        norm_diff = norm(T_solution - T_original, 'fro');

        % Guardar la matriz que presenta la mayor norma
        if norm_diff > max_norm
            max_norm = norm_diff;
            max_norm_matrix = T_solution;
        end

        % Contar las veces que la norma fue mayor a 1e-4
        if norm_diff > 1e-4 && ~large_norm_found
            count_large_norms = count_large_norms + 1;
            large_norm_found = true; % Marcar que se encontró una norma grande en esta prueba
            failed_q_randoms = [failed_q_randoms; q_random2]; % Guardar el vector q_random fallido
        end
    end
end

elapsed_time = toc; % Detener el temporizador


% Mostrar la matriz con la mayor norma y su respectiva norma
disp('Matriz con la mayor norma:');
disp(max_norm_matrix);
disp('Norma máxima:');
disp(max_norm);

% Mostrar la cantidad de veces que la norma fue mayor a 1e-4
disp('Cantidad de veces que la norma fue mayor a 1e-4:');
disp(count_large_norms);

% Mostrar el tiempo total transcurrido
disp('Tiempo total transcurrido (segundos):');
disp(elapsed_time);

% Mostrar los vectores q_random para los cuales el cálculo falló
disp('Vectores q_random para los cuales el cálculo falló:');
disp(failed_q_randoms*180/pi);