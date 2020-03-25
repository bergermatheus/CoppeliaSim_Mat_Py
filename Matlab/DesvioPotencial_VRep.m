clear all
close all
clc
%% Versão Teste Classe V-REP
                                                                                
V = VREP;
V.vConnect;

%% Carregando os objetos do cenário
V.vHandle('Pioneer_p3dx');
% V.vHandle('Pioneer_p3dx','0');

V.vObject('Disc');
% V.vObject('Disc0');
pause(2);
[p,~] = V.vGetObjPosition('Disc');
%[p,~] = V.vGetObjPosition('Disc0');


Mapa = V.vGetLaserData(1);

Mapatransp = Mapa';
%% Parametros das funções
% a= 1.5; b = 1; T = 30; w = 2*pi/T;
%% Declarando o trace
Dados=[];
[Pos.Xc1,Pos.X1, Pos.U1] = V.vGetSensorData(1);
Pos.Xd1 = zeros(8,1);

%% Inicializando variáveis para o controle
it=0;
tmax=60;
t=tic;ta=tic;
flag = 0;
k2=0;
tparcial = 15;
%% Parametros das funções
a1= 1.5; b = 1; T = 60; w = 2*pi/T;
%% Rotina da lemniscata
t=tic;ta=tic; tespera = tic;
k1=0.6;
k2=0.4;
Fk=0.002; %constante de respulsão
Fc =0.5; %constante de atração
while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        it=it+1;
        %% Robot 1
        [p,~] = V.vGetObjPosition('Disc');
        Pos.theta1= atan2(Pos.X1(2)-Pos.Xc1(2),Pos.X1(1)-Pos.Xc1(1));
        %Posição do disco;
        Pos.Xd1(1)= p(1);
        Pos.Xd1(2)= p(2);
        Pos.Xd1(7)= 0;
        Pos.Xd1(8)= 0;
        K=[cos(Pos.theta1) -0.15*sin(Pos.theta1); sin(Pos.theta1) 0.15*cos(Pos.theta1)];
       
        %Pegar informação da posição e velocidade real do robô
        [Pos.Xc1, Pos.X1, Pos.U1] = V.vGetSensorData(1);
        
        
        % Controle
        Pos.Xtil = Pos.Xd1([1 2])-Pos.X1;
        distanciaAlvoRobo = sqrt(Pos.Xtil([1])^2+Pos.Xtil([2])^2);
        %Matriz de Obstáculos    mapa
%         if toc(tespera)>1
%             tespera = tic;
            Mapa = V.vGetLaserData(1);
%         end
        % h(1)= plot(Mapa(:,1),Mapa(:,2),'.b');
        Mapatransp = Mapa';
        Obst =[Mapatransp(1,:);Mapatransp(2,:)];

        for i=1:length(Obst(1,:))
            %distancia do robo pro obstaculo mapa
            d(i) = sqrt((Obst(1,i)-Pos.X1([1]))^2+(Obst(2,i)-Pos.X1([2]))^2);
            Frep(:,i) = -Fk/d(i)^2*[Obst(:,i)-Pos.X1([1 2])]./d(i);
            if d(i)<=1.1
                flag=1;
            end

        end
        FrepSoma = sum(Frep,2);
        Fatrac = Fc*[Pos.Xd1([1 2])-Pos.X1([1 2])]./distanciaAlvoRobo;
        Fr = FrepSoma + Fatrac;
%                
        if flag == 1 
            Pos.Ud1=K\(Fr);
            flag=0;
        else
            Pos.Ud1=K\(Pos.Xd1([7 8])+k1*tanh(k2*Pos.Xtil([1 2])));
        end
                
        V.vSendControlSignals(Pos.Ud1,1);
      
    end
end

%% Comando STOP Robots
Ud = [0; 0];
V.vSendControlSignals(Ud,1);


%% Desconecta Matlab e V-REP
    V.vDisconnect;