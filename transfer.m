mu=1;
C_f=72463*2;
C_r= 92492*2;
Iz=3500;
V=30;
lf=1.07;
lr=1.53;
m=1550;
J=2392;
wf=1.6;

g=9.81;
Fz=(m*g)/4;

Cfa= 15000; %Crn=30000 olarak aldık.
theta= Cfa/(3*mu*Fz);
acri= atan(1/theta);



a0= (-2*Cfa)/(m*V);
a1= -1+((Cfa*lr- Cfa*lf)/(m*V^2));
a2= (Cfa*lr - Cfa*lf)/Iz;
a3= (-Cfa*(lf)^2 - Cfa*(lr)^2)/(Iz*V);
A=[a0 a1 ; a2 a3];
b0= Cfa/(m*V);
b1= (Cfa*lf)/Iz;
B= [b0; b1];

%yaw rate transfer function
C=[0 1];
D=0;
[num, den]= ss2tf(A,B,C,D);
G1= tf(num,den);

%sideslip angle transfer function
C2=[1 0];
[num2, den2] = ss2tf(A,B,C2,D);
G2= tf(num2 ,den2);
%pidtune(G1, 'PI')

% 
% simOut1 = sim('SA_DOB');
% pi_dob = simOut1.pidob; 
% desired = simOut1.desired;
% % 
%İkinci simülasyonu çalıştır
% simOut2 = sim('dobdesign');
% dob_fuzzy = simOut2.fuzzydob;
% %  
% figure;
% plot(pi_dob, 'r' ,'Linewidth', 3); hold on;
% plot(dob_fuzzy, 'g', 'Linewidth', 3);
% 
% grid on
% hold off;
% 
% xlabel('time (s)' , 'FontSize', 16);
% ylabel('Yaw Rate (rad/s)', 'FontSize', 16);
% title('Comparison PI-DOB and Fuzzy-DOB at 30 km/h');
% legend({'PI-DOB Control ', 'Fuzzy-DOB Control'}, 'Location', 'best');
% grid on;

s= tf('s');
%G_n= 2.2340/(0.8*s +1);
Q= 1/(0.8*s+1);

x=dcgain(G1);
G_n1= x/(0.8*s +1);
Dob_G= Q/G_n1;

Con= pidtune(G_n1, 'PI');
Kp= Con.Kp;
Ki= Con.Ki;