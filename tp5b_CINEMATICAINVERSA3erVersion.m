clc; clear; close all;

robot2;

q = [75,-15,47,159.7,-134,156]*pi/180+R.offset;
q = [-101,-77,-73.5,160,25,36]*pi/180+R.offset; 
% Vimos que para ciertos angulos para q4 como 105° o 120° ninguna fila de la matriz de 
% soluciones articulares de la funcion cinematica inversa coincidia correctamente con el vector q


% Vector de booleanos de sistemas de referencia a visualizar
sistemas = [1, 1, 1, 1, 1, 1, 1];  % (base + 6 articulaciones)

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


%% Calculamos la cinematica directa para los valores articulares elegidos
Tpunto=eye(4);
for i=1:6
    Tpunto=Tpunto*trotz(q(i))*transl(dh(i,3),0,dh(i,2))*trotx(dh(i,4));
end
fprintf("La matriz de transformacion homogenea del sistema 6 referido al sistema 0 es: \n");
disp(Tpunto);

q0 = R.offset;
q_solutions = cinInversa(R, Tpunto, 0, dh); % Si coloco 1 como tercer parámetro va a devolver la mejor solucion, sino devuelve 4
fprintf("La matriz de soluciones articulares para lograr la posicion deseada es: \n");
disp(q_solutions);

normas = zeros(1,numel(q_solutions)/6);
for j=1:numel(q_solutions)/6
    Tefector=eye(4);
    for i=1:6
        Tefector=Tefector*trotz(q_solutions(j,i)*pi/180)*transl(dh(i,3),0,dh(i,2))*trotx(dh(i,4));
    end
    normas(j) = norm(Tpunto(1:3,4)-Tefector(1:3,4));
end

fprintf("Las normas de los vectores diferencia entre el punto del efector inicial y el efector final con la cinematica inversa son: \n");
disp(normas);


%% Bucle para actualizar los sistemas de referencia con cada cambio en las articulaciones
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


%% Funcion cinematica inversa
function q = cinInversa(R, T, q_mejor, dh)
offsets = R.offset;
R.offset = zeros(6,1);
q1 = zeros(1,4); q2 = zeros(1,4); q3=zeros(1,4); q4 = zeros(1,4);
%% Sacar matrices de base y tool y trasladar al centro de la muñeca
T = invHomog(R.base.T) * T * invHomog(R.tool.T);
Tdato = T;
p_munecarespecto0 = T(1:3,4) - dh(6,2) * T(1:3,3);
p = p_munecarespecto0 - [0;0;dh(1,2)];

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
    
    syms alfa k
    
    % Paso 3: Generar la circunferencia en el plano definido por u1 y u2
    
    alfa2 = linspace(0, 2*pi, 7000);
    x2 = centro_circunferencia(1) + r * (cos(alfa2) * u1(1) + sin(alfa2) * u2(1));
    y2 = centro_circunferencia(2) + r * (cos(alfa2) * u1(2) + sin(alfa2) * u2(2));
    z2 = centro_circunferencia(3) + r * (cos(alfa2) * u1(3) + sin(alfa2) * u2(3));

    Zefector = Tdato(1:3,3);
    n = length(alfa2);
    valor_minimo = 100;
    for i=1:n
        n_pi1 = cross([x2(i);y2(i);z2(i)], p);
        n_pi2 = cross(Zefector,p-[x2(i);y2(i);z2(i)]);
        producto_escalar = abs(dot(n_pi1/norm(n_pi1), n_pi2/norm(n_pi2)));
        if (producto_escalar < valor_minimo)
            valor_minimo = producto_escalar;
            p_codo1 = [x2(i);y2(i);z2(i);1];
        end
    end
    

%% Calculo de q1 a partir de ese punto, existen dos opciones
q1(1) = atan2(p_codo1(2),p_codo1(1));
q1(2) = q1(1);
if q1(1) > 0 
    q1(3) = q1(1) - pi; 
else 
    q1(3) = q1(1) + pi;
end
q1(4) = q1(3);

% Pasar al sistema q1, tambien hay dos matrices
T1_1 = eye(4)*trotz(q1(1))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
T1_2 = eye(4)*trotz(q1(3))*trotx(dh(1,4)); % *transl(dh(1,3),0,dh(1,2))
p_codo(:,1) = invHomog(T1_1) * p_codo1;
p_codo(:,2) = invHomog(T1_2) * p_codo1;



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


%% Calculo de Q5 y Q6 = 0
q5 = zeros(1,4); q6 = zeros(1,4);
q = [q1',q2',q3',q4',q5',q6'];

punto_efector_respectoS4 = cell(1,4);

T4respecto0 = {eye(4), eye(4), eye(4), eye(4)};
for i=1:4
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
T5respecto0 = {eye(4), eye(4), eye(4), eye(4)};
for i=1:4
    for j=1:5
        if (j==1)
            T5respecto0{i}=T5respecto0{i}*trotz(q(i,j))*trotx(dh(j,4));
        else
            T5respecto0{i}=T5respecto0{i}*trotz(q(i,j))*transl(dh(j,3),0,dh(j,2))*trotx(dh(j,4));
        end
    end
    invHomog(T5respecto0{i})*T
    versorx_respectoS5=invHomog(T5respecto0{i})*T(:,1)
    if dot(cross(T5respecto0{i}(1:3,1),T(1:3,1)),T5respecto0{i}(1:3,3)) > 0
        q6(i) = atan2(versorx_respectoS5(2),versorx_respectoS5(1));
    else
        q6(i) = pi - atan2(versorx_respectoS5(2),versorx_respectoS5(1));
    end
    if q6(i) > pi
        q6(i) = - 2*pi + q6(i);
    end
    if q6(i) < - pi
        q6(i) =   2*pi - q6(i);
    end
end


q = [q1',q2',q3',q4',q5',q6'];

q = q - ones(length(q)/6,1)*offsets;

if q_mejor
    normas = zeros(2,4);
    for j=1:4
        Tefector = eye(4);
        for i=1:6
            Tefector = Tefector * trotz(q(j,i) * pi/180) * transl(dh(i,3), 0, dh(i,2)) * trotx(dh(i,4));
        end
        normas(1,j) = norm(T(1:3,4) - Tefector(1:3,4));  % Almacenas la norma
        normas(2,j) = j;  % Almacenas el índice
    end
    [~, mejor_indice] = min(normas(1,:));  % Encuentras el valor mínimo y su índice
    q = q(mejor_indice, :);  % Extraes la fila correspondiente en base al índice
end


q = q*180/pi;
% R.offset = offsets;
end

function Matriz = invHomog(T)
    Matriz = eye(4,4);
    Matriz(1:3,1:3) = T(1:3,1:3)';
    Matriz(1:3,4) = -T(1:3,1:3)'*T(1:3,4);
end