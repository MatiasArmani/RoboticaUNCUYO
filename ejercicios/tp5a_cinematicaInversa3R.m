function cinematicaInversa3R()
    clc;
    clear;
    close all;

    %% Parámetros del Robot
    d1 = 0.5; % Longitud de la primera barra vertical fija (metros)
    d2 = 1;   % Longitud de la segunda barra (metros)
    d3 = 1;   % Longitud de la tercera barra (metros)
    d4 = 0.5; % Longitud de la última barra (metros)

    %% Datos de Entrada - Posición y Orientación del Efector Final
    x = 2.0;          % Posición en x (metros)
    y = 1.0;          % Posición en y (metros)
    theta_deg = -50;   % Orientación en grados

    %% Conversión de Unidades
    theta = deg2rad(theta_deg);

    %% Cálculo de la Posición Efectiva
    x_eff = x - d4 * cos(theta);
    y_eff = y - d4 * sin(theta) - d1;

    %% Verificación de Alcance
    D = (x_eff^2 + y_eff^2 - d2^2 - d3^2) / (2 * d2 * d3);
    if abs(D) > 1
        error('La posición deseada no es alcanzable con las longitudes dadas.');
    end

    %% Soluciones para q2
    q2_elbow_up = atan2(sqrt(1 - D^2), D);
    q2_elbow_down = atan2(-sqrt(1 - D^2), D);

    %% Soluciones para q1
    phi = atan2(y_eff, x_eff);
    psi_up = atan2(d3 * sin(q2_elbow_up), d2 + d3 * cos(q2_elbow_up));
    q1_elbow_up = phi - psi_up;

    psi_down = atan2(d3 * sin(q2_elbow_down), d2 + d3 * cos(q2_elbow_down));
    q1_elbow_down = phi - psi_down;

    %% Soluciones para q3
    q3_elbow_up = theta - q1_elbow_up - q2_elbow_up;
    q3_elbow_down = theta - q1_elbow_down - q2_elbow_down;

    %% Conversión a Grados
    q1_up_deg = rad2deg(q1_elbow_up);
    q2_up_deg = rad2deg(q2_elbow_up);
    q3_up_deg = rad2deg(q3_elbow_up);

    q1_down_deg = rad2deg(q1_elbow_down);
    q2_down_deg = rad2deg(q2_elbow_down);
    q3_down_deg = rad2deg(q3_elbow_down);

    %% Verificación de Límites de las Articulaciones
    soluciones = {
        [q1_up_deg, q2_up_deg, q3_up_deg];
        [q1_down_deg, q2_down_deg, q3_down_deg];
    };

    fprintf('Soluciones de Cinemática Inversa:\n');
    for i = 1:length(soluciones)
        q = soluciones{i};
        if all(q >= -180) && all(q <= 180)
            fprintf('Solución %d:\n', i);
            fprintf('q1 = %.2f°\n', q(1));
            fprintf('q2 = %.2f°\n', q(2));
            fprintf('q3 = %.2f°\n\n', q(3));
        else
            fprintf('Solución %d está fuera de los límites de las articulaciones.\n\n', i);
        end
    end

    %% Muestreo de Soluciones
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Configuraciones del Robot 3R');

    colors = ['r', 'b'];
    for i = 1:length(soluciones)
        q = soluciones{i};
        if all(q >= -180) && all(q <= 180)
            % Convertir a radianes
            q_rad = deg2rad(q);
            % Cálculo de las posiciones de las articulaciones
            base = [0, 0];
            base_static = base + [0, d1]; % Fin de la primer barra vertical
            joint1 = base_static;
            joint2 = joint1 + d2 * [cos(q_rad(1)), sin(q_rad(1))];
            joint3 = joint2 + d3 * [cos(q_rad(1)+q_rad(2)), sin(q_rad(1)+q_rad(2))];
            end_effector = joint3 + d4 * [cos(q_rad(1)+q_rad(2)+q_rad(3)), sin(q_rad(1)+q_rad(2)+q_rad(3))];

            % Dibujar primer barra vertical
            plot([base(1), base_static(1)], [base(2), base_static(2)], 'k-', 'LineWidth', 2);

            % Dibujar las barras dinámicas
            plot([joint1(1), joint2(1)], [joint1(2), joint2(2)], 'Color', colors(i), 'LineWidth', 2);
            plot([joint2(1), joint3(1)], [joint2(2), joint3(2)], 'Color', colors(i), 'LineWidth', 2);
            plot([joint3(1), end_effector(1)], [joint3(2), end_effector(2)], 'Color', colors(i), 'LineWidth', 2);

            % Dibujar las articulaciones
            plot(base(1), base(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');          % Base
            plot(base_static(1), base_static(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Fin de la barra fija
            plot(joint1(1), joint1(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');      % Joint1
            plot(joint2(1), joint2(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');      % Joint2
            plot(joint3(1), joint3(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');      % Joint3
            plot(end_effector(1), end_effector(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Efector Final

            % Etiquetas
            text(base(1), base(2), ' Base', 'FontSize', 10);
            text(end_effector(1), end_effector(2), [' Efector'], 'FontSize', 10);
        end
    end

    % Dibujar la posición deseada del efector
    plot(x, y, 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;
end