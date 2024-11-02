function q = cinInversa(R, T, q_mejor, dh, q0)
offsets = R.offset;
R.offset = zeros(6,1);
q1 = zeros(1,4); q2 = zeros(1,4); q3=zeros(1,4); q4 = zeros(1,4);
%% Sacar matrices de base y tool y trasladar al centro de la muñeca
T = invHomog(R.base.T) * T * invHomog(R.tool.T);
Tdato = T;
p_munecarespecto0 = T(1:3,4) - dh(6,2) * T(1:3,3);
p = p_munecarespecto0 - [0;0;dh(1,2)];  % Punto muñeca respecto punto del hombro

%% Calculo de punto sobre la circunferencia en la que puede quedar el codo utilizando parametro alfa

% Definir el plano de la circunferencia intersección
    u1 = cross(p, [0;0;1]);  % Primer versor en el plano de la circunferencia
    u1 = u1 / norm(u1);
    u2 = cross(u1, p);       % Segundo versor perpendicular a u1 y p
    u2 = u2 / norm(u2);
    
    % Definir el radio y el centro de la circunferencia
    Radio = sqrt(p(1)^2 + p(2)^2 + p(3)^2);  % Distancia al centro
    r = sqrt(dh(3,2)^2 - (Radio/2)^2);       % Radio de la circunferencia

    % Centro de la circunferencia
    centro_circunferencia = p / 2;  % Punto medio del vector que une hombro y muñeca
    
    % Paso 3: Generar la circunferencia en el plano definido por u1 y u2
    
    alfa2 = linspace(0, 2*pi, 7000);
    x2 = centro_circunferencia(1) + r * (cos(alfa2) * u1(1) + sin(alfa2) * u2(1));
    y2 = centro_circunferencia(2) + r * (cos(alfa2) * u1(2) + sin(alfa2) * u2(2));
    z2 = centro_circunferencia(3) + r * (cos(alfa2) * u1(3) + sin(alfa2) * u2(3));

Zefector = Tdato(1:3,3);
n = length(alfa2);
producto_escalar = zeros(1,n);

% Calcular el producto escalar para todos los puntos
for i=1:n
    n_pi1 = cross([x2(i);y2(i);z2(i)], p);
    n_pi2 = cross(Zefector,p-[x2(i);y2(i);z2(i)]);
    producto_escalar(i) = abs(dot(n_pi1/norm(n_pi1), n_pi2/norm(n_pi2)));
end

% Inicializar variables para los mínimos locales
minimos_locales = [];
indices_minimos_locales = [];

% Detectar cambios de pendiente y encontrar los mínimos locales
for i = 2:n-1
    if producto_escalar(i-1) > producto_escalar(i) && producto_escalar(i) < producto_escalar(i+1)
        minimos_locales = [minimos_locales, producto_escalar(i)];
        indices_minimos_locales = [indices_minimos_locales, i];
    end
end

% Verificar que se encontraron al menos dos mínimos locales
if length(minimos_locales) >= 2
    % Elegir los dos mínimos más bajos
    [~, idx_min_1] = min(minimos_locales);
    minimos_locales(idx_min_1) = inf;  % Ignorar el primer mínimo encontrado
    [~, idx_min_2] = min(minimos_locales);
    
    % Mostrar los dos mínimos encontrados
    p_codo1 = [x2(indices_minimos_locales(idx_min_1)); y2(indices_minimos_locales(idx_min_1)); z2(indices_minimos_locales(idx_min_1)); 1];
    p_codo2 = [x2(indices_minimos_locales(idx_min_2)); y2(indices_minimos_locales(idx_min_2)); z2(indices_minimos_locales(idx_min_2)); 1];
    
    % disp('Primer mínimo en el índice:');
    % disp(indices_minimos_locales(idx_min_1));
    % disp('Segundo mínimo en el índice:');
    % disp(indices_minimos_locales(idx_min_2));
else
    [~, idx_min_1] = min(minimos_locales);
    % Mostrar los dos mínimos encontrados
    p_codo1 = [x2(indices_minimos_locales(idx_min_1)); y2(indices_minimos_locales(idx_min_1)); z2(indices_minimos_locales(idx_min_1)); 1];
    p_codo2 = [x2(1); y2(1); z2(1); 1];
    % disp('No se encontraron dos mínimos locales');
end
%Graficar la curva
% figure;
% i = 1:1:7000;
% plot(i,producto_escalar(1:7000),'r');
% grid on;
% hold on;

%% Calculo de q1 a partir de ese punto, existen dos opciones
q1(1) = atan2(p_codo1(2),p_codo1(1));
q1(2) = q1(1);
q1(5) = atan2(p_codo2(2),p_codo2(1));
q1(6) = q1(5);
if q1(1) > 0 
    q1(3) = q1(1) - pi; 
else 
    q1(3) = q1(1) + pi;
end
q1(4) = q1(3);

% Calculo de las otras 4 q
if q1(5) > 0
    q1(7) = q1(5) - pi;
else
    q1(7) = q1(5) - pi;
end
q1(8) = q1(7);

% Pasar al sistema q1, tambien hay dos matrices
T1_1 = eye(4)*trotz(q1(1))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
T1_2 = eye(4)*trotz(q1(3))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
p_codo(:,1) = invHomog(T1_1) * p_codo1;
p_codo(:,2) = invHomog(T1_2) * p_codo1;

% Calculos para las otras 4 soluciones
T1_3 = eye(4)*trotz(q1(5))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
T1_4 = eye(4)*trotz(q1(7))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
p_codo4extra(:,1) = invHomog(T1_3) * p_codo2;
p_codo4extra(:,2) = invHomog(T1_4) * p_codo2;


temp1 = atan2(p_codo(2,1),p_codo(1,1));
if (temp1 > pi/2) || (temp1 < -pi/2)
    q2(1) = temp1;
else
    q2(1) = - pi/2 + temp1;
end
q2(2) = q2(1);

temp2 = atan2(p_codo(2,2),p_codo(1,2));
if (temp1 > pi/2) || (temp1 < -pi/2)
    q2(3) = temp2;
else
    q2(3) = - pi/2 + temp2;
end
q2(4)=q2(3);

%Calculos para las otras 4 soluciones

temp12 = atan2(p_codo4extra(2,1),p_codo4extra(1,1));
if (temp12 > pi/2) || (temp12 < -pi/2)
    q2(5) = temp12;
else
    q2(5) = - pi/2 + temp12;
end
q2(6) = q2(5);

temp22 = atan2(p_codo4extra(2,2),p_codo4extra(1,2));
if (temp12 > pi/2) || (temp12 < -pi/2)
    q2(7) = temp22;
else
    q2(7) = - pi/2 + temp22;
end
q2(8)=q2(7);

%% Calculo de q3
T12_1 = eye(4)*trotz(q2(1))*trotx(dh(2,4));
T12_2 = eye(4)*trotz(q2(3))*trotx(dh(2,4));
punto_muneca_respectoHombro = [p;1];
punto_muneca_respectoS2_1 = invHomog(T12_1)*invHomog(T1_1)*punto_muneca_respectoHombro;
punto_muneca_respectoS2_2 = invHomog(T12_2)*invHomog(T1_2)*punto_muneca_respectoHombro;

q3(1) = atan2(punto_muneca_respectoS2_1(2),punto_muneca_respectoS2_1(1));
if q3(1) > 0
    q3(2) = q3(1) - pi;
else
    q3(2) = q3(1) + pi;
end
q3(3) = atan2(punto_muneca_respectoS2_2(2),punto_muneca_respectoS2_2(1));
if q3(3) > 0
    q3(4) = q3(3) - pi;
else
    q3(4) = q3(3) + pi;
end

% Calculo de las otras 4 soluciones:
T12_3 = eye(4)*trotz(q2(5))*trotx(dh(2,4));
T12_4 = eye(4)*trotz(q2(7))*trotx(dh(2,4));
punto_muneca_respectoHombro = [p;1];
punto_muneca_respectoS2_3 = invHomog(T12_3)*invHomog(T1_3)*punto_muneca_respectoHombro;
punto_muneca_respectoS2_4 = invHomog(T12_4)*invHomog(T1_4)*punto_muneca_respectoHombro;

q3(5) = atan2(punto_muneca_respectoS2_3(2),punto_muneca_respectoS2_3(1));
if q3(5) > 0
    q3(6) = q3(5) - pi;
else
    q3(6) = q3(5) + pi;
end
q3(7) = atan2(punto_muneca_respectoS2_4(2),punto_muneca_respectoS2_4(1));
if q3(7) > 0
    q3(8) = q3(7) - pi;
else
    q3(8) = q3(7) + pi;
end

%% Calculo de q4
T23_1 = eye(4)*trotz(q3(1))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_2 = eye(4)*trotz(q3(2))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_3 = eye(4)*trotz(q3(3))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_4 = eye(4)*trotz(q3(4))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));

punto_muneca_respectoS3_1 = invHomog(T23_1)*punto_muneca_respectoS2_1;
punto_muneca_respectoS3_2 = invHomog(T23_2)*punto_muneca_respectoS2_1;
punto_muneca_respectoS3_3 = invHomog(T23_3)*punto_muneca_respectoS2_2;
punto_muneca_respectoS3_4 = invHomog(T23_4)*punto_muneca_respectoS2_2;

q4(1) = atan2(punto_muneca_respectoS3_1(2),punto_muneca_respectoS3_1(1));
q4(2) = atan2(punto_muneca_respectoS3_2(2),punto_muneca_respectoS3_2(1));
q4(3) = atan2(punto_muneca_respectoS3_3(2),punto_muneca_respectoS3_3(1));
q4(4) = atan2(punto_muneca_respectoS3_4(2),punto_muneca_respectoS3_4(1));

% Calculo de las otras 4 soluciones
T23_5 = eye(4)*trotz(q3(5))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_6 = eye(4)*trotz(q3(6))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_7 = eye(4)*trotz(q3(7))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));
T23_8 = eye(4)*trotz(q3(8))*transl(dh(3,3),0,dh(3,2))*trotx(dh(3,4));

punto_muneca_respectoS3_5 = invHomog(T23_5)*punto_muneca_respectoS2_3;
punto_muneca_respectoS3_6 = invHomog(T23_6)*punto_muneca_respectoS2_3;
punto_muneca_respectoS3_7 = invHomog(T23_7)*punto_muneca_respectoS2_4;
punto_muneca_respectoS3_8 = invHomog(T23_8)*punto_muneca_respectoS2_4;

q4(5) = atan2(punto_muneca_respectoS3_5(2),punto_muneca_respectoS3_5(1));
q4(6) = atan2(punto_muneca_respectoS3_6(2),punto_muneca_respectoS3_6(1));
q4(7) = atan2(punto_muneca_respectoS3_7(2),punto_muneca_respectoS3_7(1));
q4(8) = atan2(punto_muneca_respectoS3_8(2),punto_muneca_respectoS3_8(1));


%% Calculo de Q5 y Q6 = 0
q5 = zeros(1,8); q6 = zeros(1,8);
q = [q1',q2',q3',q4',q5',q6'];

punto_efector_respectoS4 = cell(1,4);

T4respecto0 = {eye(4), eye(4), eye(4), eye(4), eye(4), eye(4), eye(4), eye(4)};
for i=1:8
    for j=1:4
        if (j==1)
            T4respecto0{i}=T4respecto0{i}*trotz(q(i,j))*trotx(dh(j,4));
        else
            T4respecto0{i}=T4respecto0{i}*trotz(q(i,j))*transl(dh(j,3),0,dh(j,2))*trotx(dh(j,4));
        end
    end
    punto_efector_respectoS4{i} = invHomog(T4respecto0{i})*(T(1:4,4)-[0;0;dh(1,2);0]);
    temp3 = atan2(punto_efector_respectoS4{i}(2),punto_efector_respectoS4{i}(1));
    q5(i) = pi/2 + temp3;
    if q5(i) > pi
        q5(i) = q5(i) - 2*pi;
    end
    if q5(i) < -pi
        q5(i) = q5(i) + 2*pi;
    end
end
q(:,5) = q5';
T5respecto0 = {eye(4), eye(4), eye(4), eye(4), eye(4), eye(4), eye(4), eye(4)};
for i=1:8
    for j=1:5
        if (j==1)
            T5respecto0{i}=T5respecto0{i}*trotz(q(i,j))*trotx(dh(j,4));
        else
            T5respecto0{i}=T5respecto0{i}*trotz(q(i,j))*transl(dh(j,3),0,dh(j,2))*trotx(dh(j,4));
        end
    end
    invHomog(T5respecto0{i})*T;
    versorx_respectoS5=invHomog(T5respecto0{i})*T(:,1);
    q6(i) = atan2(versorx_respectoS5(2),versorx_respectoS5(1));
end

q = [q1',q2',q3',q4',q5',q6'];

q = q - ones(length(q)/length(q1),1)*offsets;


if q_mejor
    normas = zeros(2,8);
    for j=1:8
        % Normalizar los ángulos para que estén en el rango de -pi a pi
        q_normalizado = (q(j,1:5) - q0(1:5));
        normas(1,j) = norm(q_normalizado);
        normas(2,j) = j;
    end
    [~, mejor_indice] = min(normas(1,:));
    q = q(mejor_indice,:);
end

% q = q*180/pi;
R.offset = offsets;

function Matriz = invHomog(T)
        Matriz = eye(4,4);
        Matriz(1:3,1:3) = T(1:3,1:3)';
        Matriz(1:3,4) = -T(1:3,1:3)'*T(1:3,4);
end
end

