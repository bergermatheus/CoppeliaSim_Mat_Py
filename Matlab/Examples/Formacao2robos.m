%%Controle formação 2 robôs;

clear all
close all
clc                                                                               
V = VREP;
V.vConnect;

%% Carregando os objetos do cenário

V.vHandle('Pioneer_p3dx');
V.vHandle('Pioneer_p3dx','0');

%% Declarando o trace

Dados=[];
[Pos.Xc1,Pos.X1, Pos.U1] = V.vGetSensorData(1);
[Pos.Xc2,Pos.X2, Pos.U2] = V.vGetSensorData(2);
Pos.Xd1 = zeros(8,1);
Pos.Xd2 = zeros(8,1);

%% Inicializando variáveis para o controle

it=0;
tmax=60;
t=tic;ta=tic;
Dados = [];
k1 = 0.8;
k2 = 0.8;
T = 40; w = 2*pi/T;

%% Plotar Rastro

while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        
        %% Controle

        Xf    = 3*cos(w*toc(t)); 
%         Xf    = 2*sin(0.5*w*toc(t));
        Xfr   = (Pos.X1(1)+Pos.X2(1))/2;
        dXf   = -3*w*sin(w*toc(t)); 
%         dXf   = w*cos(0.5*w*toc(t));

        Yf    = 3*sin(w*toc(t)); 
%         Yf    = 2*sin(w*toc(t)); 
        Yfr   = (Pos.X1(2)+Pos.X2(2))/2;
        dYf   = 3*w*cos(w*toc(t)); 
%         dYf   = 2*w*cos(w*toc(t));

        Lf    = 1.5; % Distância entre a formação
        Lfr   = sqrt((Pos.X1(1)-Pos.X2(1))^2+(Pos.X1(2)-Pos.X2(2))^2);
        dLf   = 0;
        
        Psif  = atan2(Yf,Xf)+3*pi/4;
        Psifr = atan2(Pos.X1(2)-Pos.X2(2),Pos.X1(1)-Pos.X2(1));
        dPsif = (1/(Xf^2 +Yf^2))*(dYf*Xf-Yf*dXf);
               
        Pos.Xd1(1) = Xf + (Lf/2)*cos(Psif);
        Pos.Xd1(2) = Yf + (Lf/2)*sin(Psif);
        Pos.Xd1(7) = dXf/2 - (Lf/2)*sin(Psif)*dPsif - cos(Psif/2)*dLf;
        Pos.Xd1(8) = dYf/2 + (Lf/2)*cos(Psif)*dPsif - sin(Psif/2)*dLf;
        
        Pos.Xd2(1) = Xf - (Lf/2)*cos(Psif);
        Pos.Xd2(2) = Yf - (Lf/2)*sin(Psif);
        Pos.Xd2(7) = dXf/2 + (Lf/2)*sin(Psif)*dPsif + cos(Psif/2)*dLf;
        Pos.Xd2(8) = dYf/2 - (Lf/2)*cos(Psif)*dPsif + sin(Psif/2)*dLf;
        
        
        
        %% Robot 1
        
        [Pos.Xc1, Pos.X1, Pos.U1] = V.vGetSensorData(1);
        Pos.theta1= atan2(Pos.X1(2)-Pos.Xc1(2),Pos.X1(1)-Pos.Xc1(1));
        K1=[cos(Pos.theta1) -0.15*sin(Pos.theta1); sin(Pos.theta1) 0.15*cos(Pos.theta1)];
        Pos.Xtil1 = Pos.Xd1([1 2])-Pos.X1;
        Pos.Ud1=K1\(Pos.Xd1([7 8])+k1*tanh(k2*Pos.Xtil1([1 2])));
        V.vSendControlSignals(Pos.Ud1,1);
        
        %% Robot 2
        
        [Pos.Xc2, Pos.X2, Pos.U2] = V.vGetSensorData(2);
        Pos.theta2= atan2(Pos.X2(2)-Pos.Xc2(2),Pos.X2(1)-Pos.Xc2(1));
        K2=[cos(Pos.theta2) -0.15*sin(Pos.theta2); sin(Pos.theta2) 0.15*cos(Pos.theta2)];
        Pos.Xtil2 = Pos.Xd2([1 2])-Pos.X2;
        Pos.Ud2=K2\(Pos.Xd2([7 8])+k1*tanh(k2*Pos.Xtil2([1 2])));
        V.vSendControlSignals(Pos.Ud2,2);
        
       
    end
end


%% Comando STOP Robots

Ud = [0; 0];
V.vSendControlSignals(Ud,1);
V.vSendControlSignals(Ud,2);


%% Desconecta Matlab e V-REP

V.vDisconnect;
