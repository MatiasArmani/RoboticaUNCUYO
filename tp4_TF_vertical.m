clc; close all; clear;

robot;

R.plot([0,0,0,0,0,0],'trail',{'r','linewidth',1},'delay',0.001);


N = 100;
q1 = jtraj(q,[0,0,R.qlim(3,2),0,0,0],N);
q2 = jtraj([0,0,R.qlim(3,2),0,0,0],[0,0,R.qlim(3,2),R.qlim(4,2),0,0],N);
q3 = jtraj([0,0,R.qlim(3,2),R.qlim(4,2),0,0],[0,0,R.qlim(3,1),R.qlim(4,2),0,0],N);
q4 = jtraj([0,0,R.qlim(3,1),R.qlim(4,2),0,0],[0,0,R.qlim(3,2),R.qlim(4,2),0,0],N);
q5 = jtraj([0,0,R.qlim(3,2),R.qlim(4,2),0,0],[0,0,R.qlim(3,2),0,0,0],N);
q6 = jtraj([0,0,R.qlim(3,2),0,0,0],[0,0,R.qlim(3,1),0,0,0],N);
q7 = jtraj([0,0,R.qlim(3,1),0,0,0],[0,0,R.qlim(3,1),R.qlim(4,1),0,0],N);
q8 = jtraj([0,0,R.qlim(3,1),R.qlim(4,1),0,0],[0,0,R.qlim(3,2),R.qlim(4,1),0,0],N);


qTot = [q1;q2;q3;q4;q5;q6;q7;q8];
R.animate(qTot);
