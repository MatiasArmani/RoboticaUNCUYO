clc; close all; clear;

robot;

R.plot([0,pi/2,0,0,0,0],'trail',{'r','linewidth',1},'delay',0.001);

N = 100;
q1 = jtraj([0,pi/2,0,0,0,0],[0,pi/2,R.qlim(3,2),0,0,0],N);
q2 = jtraj([0,pi/2,R.qlim(3,2),0,0,0],[0,pi/2,R.qlim(3,2),R.qlim(4,2),0,0],N);
q3 = jtraj([0,pi/2,R.qlim(3,2),R.qlim(4,2),0,0],[0,pi/2,R.qlim(3,1),R.qlim(4,2),0,0],N);
q4 = jtraj([0,pi/2,R.qlim(3,1),R.qlim(4,2),0,0],[0,pi/2,R.qlim(3,2),R.qlim(4,2),0,0],N);
q5 = jtraj([0,pi/2,R.qlim(3,2),R.qlim(4,2),0,0],[0,pi/2,R.qlim(3,2),0,0,0],N);
q6 = jtraj([0,pi/2,R.qlim(3,2),0,0,0],[0,pi/2,R.qlim(3,1),0,0,0],N);
q7 = jtraj([0,pi/2,R.qlim(3,1),0,0,0],[0,pi/2,R.qlim(3,1),R.qlim(4,1),0,0],N);
q8 = jtraj([0,pi/2,R.qlim(3,1),R.qlim(4,1),0,0],[0,pi/2,R.qlim(3,2),R.qlim(4,1),0,0],N);
q9 = jtraj([0,pi/2,R.qlim(3,2),R.qlim(4,1),0,0],[0,-pi/2,R.qlim(3,2),R.qlim(4,1),0,0],N);
q11 = jtraj([0,-pi/2,R.qlim(3,2),0,0,0],[0,-pi/2,0,0,0,0],N);
q22 = jtraj([0,-pi/2,R.qlim(3,2),R.qlim(4,2),0,0],[0,-pi/2,R.qlim(3,2),0,0,0],N);
q33 = jtraj([0,-pi/2,R.qlim(3,1),R.qlim(4,2),0,0],[0,-pi/2,R.qlim(3,2),R.qlim(4,2),0,0],N);
q44 = jtraj([0,-pi/2,R.qlim(3,2),R.qlim(4,2),0,0],[0,-pi/2,R.qlim(3,1),R.qlim(4,2),0,0],N);
q55 = jtraj([0,-pi/2,R.qlim(3,2),0,0,0],[0,-pi/2,R.qlim(3,2),R.qlim(4,2),0,0],N);
q66 = jtraj([0,-pi/2,R.qlim(3,1),0,0,0],[0,-pi/2,R.qlim(3,2),0,0,0],N);
q77 = jtraj([0,-pi/2,R.qlim(3,1),R.qlim(4,1),0,0],[0,-pi/2,R.qlim(3,1),0,0,0],N);
q88 = jtraj([0,-pi/2,R.qlim(3,2),R.qlim(4,1),0,0],[0,-pi/2,R.qlim(3,1),R.qlim(4,1),0,0],N);


qTot = [q1;q2;q3;q4;q5;q6;q7;q8;q9;q88;q77;q66;q55;q44;q33;q22;q11];
R.animate(qTot);
