clc, clear, close all

dh = [
    0   0.450    0.075   -pi/2    0;
    0       0    0.300       0    0;
    0       0    0.075   -pi/2    0;
    0   0.320        0    pi/2    0;
    0       0        0   -pi/2    0;
    0   0.008        0       0    0];

R1 = SerialLink(dh,'name','Paint Mate 200iA 1');
R2 = SerialLink(dh,'name','Paint Mate 200iA 2');
R3 = SerialLink(dh,'name','Paint Mate 200iA 3');
R4 = SerialLink(dh,'name','Paint Mate 200iA 4');

q1 = [0,0,0,0,0,0];
q2 = [pi/4,-pi/2,0,0,0,0];
q3 = [pi/5,-2*pi/5,-pi/10,pi/2,3*pi/10,-pi/2];
q4 = [-0.61, -0.15, -0.30, 1.40, 1.90, -1.40];

listMatrix1 = matricesCinematicaDirecta(q1, dh);
listMatrix2 = matricesCinematicaDirecta(q2, dh);
listMatrix3 = matricesCinematicaDirecta(q3, dh);
listMatrix4 = matricesCinematicaDirecta(q4, dh);

disp('Matrices de transformacion para q1');
imprimirMatrices(listMatrix1);
disp('Matrices de transformacion para q2');
imprimirMatrices(listMatrix2);
disp('Matrices de transformacion para q3');
imprimirMatrices(listMatrix3);
disp('Matrices de transformacion para q4');
imprimirMatrices(listMatrix4);


% Subplot 1
subplot(2,2,1);
R1.plot(q1, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
title('Posición q1');

% Subplot 2
subplot(2,2,2);
R2.plot(q2, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
title('Posición q2');

% Subplot 3
subplot(2,2,3);
R3.plot(q3, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
title('Posición q3');

% Subplot 4
subplot(2,2,4);
R4.plot(q4, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
title('Posición q4');


function listMatrix = matricesCinematicaDirecta(q, dh)
    n = length(q)+1;
    prod = eye(4);
    listMatrix = zeros(4, 4, n); 
    for i = (1:n-1)
        listMatrix(:,:,i) = trotz(q(i)) * transl(dh(i,3),0,dh(i,2)) * trotx(dh(i,4));
        prod = prod * listMatrix(:,:,i);
    end
    listMatrix(:,:,n) = prod;
end

function imprimirMatrices(listMatrix)
    [~,~,n] = size(listMatrix);
    for i = 1:n-1
        fprintf('%.0fM%.0f es la posición y orientación del sistema %d respecto del %d:\n',i-1 , i, i, i-1);
        disp(listMatrix(:,:,i));
    end
    fprintf('%.0fM%.0f es la posición y orientación del sistema %d respecto del %d:\n', 0,  n-1, n-1, 0);
    disp(listMatrix(:,:,n));
end