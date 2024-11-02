clc, clear, close all
fprintf('1: Jacobiano y determinante\n')
fprintf('2: Jacobiano y determinante. Con VPA y sin simplify\n')
fprintf('3: Singularidades\n')
OPT = input('\nOpción: ');

% DH del robot (robot2.m)
dh = [
    0   0.150    0    pi/2;
    0   0        0   -pi/2;
    0   0.500    0    pi/2;
    0   0        0.500  -pi/2;
    0   0        0    pi/2;
    0   0.050    0       0];

% Crear modelo del robot
R = SerialLink(dh);

if OPT == 1
    % Jacobiano simbólico
    fprintf('> Cálculo del Jacobiano y su determinante\n\n')
    
    syms q1 q2 q3 q4 q5 q6 real
    q = [q1 q2 q3 q4 q5 q6];
    
    % Jacobiano y determinante (solo simbólico en la opción 1)
    J = simplify(R.jacob0(q));
    DJ = simplify(det(J));  % Determinante del Jacobiano completo (6x6)
    
    fprintf('Jacobiano J =\n')
    disp(J)
    
    fprintf('Determinante del Jacobiano completo: det(J) = ')
    disp(DJ)

    pause

elseif OPT == 2
    % Cálculo del Jacobiano con vpa (aritmética de precisión variable)
    fprintf('> Cálculo del Jacobiano con vpa\n\n')
    
    syms q1 q2 q3 q4 q5 q6 real
    q = [q1 q2 q3 q4 q5 q6];
    
    % Jacobiano
    J = R.jacob0(q);
    DJ_vpa = vpa(det(J));  % Determinante con vpa
    
    fprintf('Jacobiano J =\n')
    disp(J)
    
    fprintf('Determinante del Jacobiano completo con vpa: det(J) = ')
    disp(DJ_vpa)

    pause

elseif OPT == 3
    % Identificación de singularidades (cálculo numérico en puntos específicos)
    fprintf('> Identificación de Singularidades\n\n')
    
    rng(42);  % Fijamos la semilla para reproducibilidad
    
    % Singularidad de muñeca (q5 = pi/2, resto aleatorio)
    q_singular1 = [randn()*0.1, randn()*0.1, randn()*0.1, randn()*0.1, pi/2, randn()*0.1];
    
    % Singularidad de hombro (q2 = 0, resto aleatorio)
    q_singular3 = [randn()*0.1, 0, randn()*0.1, randn()*0.1, randn()*0.1, randn()*0.1];
    
    % Singularidad de codo en q4 (q4 = pi/2, resto aleatorio)
    q_singular4 = [randn()*0.1, randn()*0.1, randn()*0.1, pi/2, randn()*0.1, randn()*0.1];


    % Evaluar el Jacobiano y determinante para cada configuración
    % Singularidad de muñeca (q5 = 0)
    J_test1 = R.jacob0(q_singular1);
    DJ_test1 = det(J_test1);  % Determinante del Jacobiano completo (6x6)
    fprintf('Determinante en singularidad de muñeca (q5 = pi/2): %e\n', DJ_test1);
    
    % Singularidad de hombro (q2 = 0)
    J_test3 = R.jacob0(q_singular3);
    DJ_test3 = det(J_test3);  % Determinante del Jacobiano completo (6x6)
    fprintf('Determinante en singularidad de hombro (q2 = 0): %e\n', DJ_test3);
    
    % Singularidad de codo en q4 (q4 = pi/2)
    J_test4 = R.jacob0(q_singular4);
    DJ_test4 = det(J_test4);  % Determinante del Jacobiano completo (6x6)
    fprintf('Determinante en singularidad de codo (q4 = pi/2): %e\n', DJ_test4);

    pause
end