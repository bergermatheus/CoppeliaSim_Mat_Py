clear all
close all
clc
%% Versão Teste Classe V-REP
V = VREP;
V.vConnect;

%% Carregando os objetos do cenário
V.vHandle('Pioneer_p3dx');

V.vObject('Disc');
pause(2);
%% Get Destination point and Robot Position
[Goal,~] = V.vGetObjPosition('Disc');


%% Declarando a figura
figure(1)
hold on
axis([-6,6,-6,6])
Map = V.vGetLaserData(1);

h(1)= plot(Map(:,1),Map(:,2),'.b');
hold on

%% Declarando o trace
Dados=[];
%% Inicializando variáveis para o controle
it=0;
tmax=30;
t=tic;ta=tic;
flag = 0;

%% Rotina Principal
while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        
        [Goal,~] = V.vGetObjPosition('Disc');
        
        %% Posição desejada
        Pos.Xd(1)= Goal(1);
        Pos.Xd(2)= Goal(2);
        Pos.Xd(7)= 0;
        Pos.Xd(8)= 0;
        
        %Pegar informação da posição e velocidade real do robô
        [Pos.Xc, Pos.X, Pos.U] = V.vGetSensorData(1);
        
        % Orientação
        Pos.theta= atan2(Pos.X(2)-Pos.Xc(2),Pos.X(1)-Pos.Xc(1));
        
        K=[cos(Pos.theta) -0.15*sin(Pos.theta); sin(Pos.theta) 0.15*cos(Pos.theta)];
        
        % Control System
        Pos.Xtil = Pos.Xd([1 2])'-Pos.X([1 2]);
        Pos.Ud = K\(Pos.Xd([7 8])+0.7*tanh(0.5*Pos.Xtil([1 2])));
        
        delete(h)
        Map = V.vGetLaserData(1);
        h(1)= plot(Map(:,1),Map(:,2),'.b');
        drawnow
        % Send Control Signal to VREP
        V.vSendControlSignals(Pos.Ud,1);
        
        
    end
end

%% Comando STOP Robots
Ud = [0; 0];
V.vSendControlSignals(Ud,1);


%% Desconecta Matlab e V-REP
V.vDisconnect;