d1 = 0.150;
d6 = 0.150;
d3 = 0.500;
a4 = 0.500;

dEspatula = 0.0;

hBase = 0; 

dh = [
    0   d1    0    pi/2   0;
    0    0    0   -pi/2   0;
    0   d3    0    pi/2   0;
    0    0   a4   -pi/2   0;
    0    0    0    pi/2   0;
    0   d6    0       0   0];

q = [0,0,0,0,0,0];

R = SerialLink(dh,'name','Robot Heladero','plotopt',{'scale',0.8,'jointdiam',1.4,'jointcolor',[0 0.3 0.5],'name','tilesize',0.2,'tile1color',[1 1 1],'noname','lightpos',[0 0 10]});

% Límites articulares basados en las restricciones físicas del robot
R.qlim(1,1:2) = [-180, 180]*pi/180;
R.qlim(2,1:2) = [ -60,  240]*pi/180;
R.qlim(3,1:2) = [ -180, 180]*pi/180;
R.qlim(4,1:2) = [-150, 150]*pi/180;
R.qlim(5,1:2) = [ -150,  150]*pi/180;
R.qlim(6,1:2) = [-150, 150]*pi/180;

% Configuración de la base del robot en el entorno de trabajo
R.base = transl(0, 0, hBase);  % Posición de la base

% Configuración de la herramienta, representando la cuchara
R.tool = transl(0, 0, dEspatula);  % Ajuste según la cuchara

% Offsets articulares
R.offset = [0,-pi/2,pi/2,pi/2,pi/2,0];