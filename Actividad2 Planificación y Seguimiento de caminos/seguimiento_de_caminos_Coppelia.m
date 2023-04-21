%clear all
close all
clc

%% Inicio de conexion
sim=remApi('remoteApi'); % usando el prototipo de función (remoteApiProto.m)
sim.simxFinish(-1); % Cerrar las conexiones anteriores en caso de que exista una
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%% Verificación de la conexión
if (clientID>-1)
    disp('Conexión con Coppelia iniciada');
    %% Codigo de control
    % Preparación
    % Se crean los Hanldes correspondientes al motor, sensor ultrasonico,
    % el robot y la referencia
    [~, left_motor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [~, right_motor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    [~, pioneer_block]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx',sim.simx_opmode_blocking);

    [~, actualPos]=sim.simxGetObjectPosition(clientID,pioneer_block, ...
        -1,sim.simx_opmode_streaming);
    [~,angulo]=sim.simxGetObjectOrientation(clientID, pioneer_block, ...
        -1, sim.simx_opmode_streaming);

 %importar mapa
 archivo = "mapaactividad2.xlsx";
 BinaryMap = flipud(readmatrix(archivo));


 %Creación del mapa
if exist('BinaryMap','var') == 0 
    BinaryMap = makemap(1200)
end

figure()
%map = binaryOccupancyMap(sourcemap,resolution)
map = binaryOccupancyMap(rot90(transpose(BinaryMap)),1200/12);
show(map)

nodes = 100;
tic
planner = mobileRobotPRM(map,nodes)
toc

startLocation = [0.5 0.5];
endLocation = [6 6];
tic
path = findpath(planner,startLocation,endLocation);
toc
figure()
show(planner)

%% ChatGPT prompt:
% Create a program to simulate a differential robot in matlab

%Position
%Control del robot
desPos = path-6;
numPos = size(desPos, 1);

next_i=1;
next_j=1;

xd = desPos(next_j,1);
yd = desPos(next_j,2);

% Define robot parameters
wheel_radius = 0.0975; % radius of wheels (m)
wheelbase = 0.381; % distance between wheels (m) (l)
dt = 0.01; % time step (s)
t = 0; % Total time (s)
v = 1; % maximum linear velocity (m/s)
w = pi/2; % maximum angular velocity (rad/s)

% Define desired position
kpr = 5;
kpt = 8;
vmax = 1;
i = 1;

xe = actualPos(1)-xd;
ye = actualPos(2)-yd;

while next_j <= numPos
    xd = desPos(next_j,1);
    yd = desPos(next_j,2);
    xe = actualPos(1)-xd;
    ye = actualPos(2)-yd;
    % Simulate robot motion
    d = sqrt((actualPos(1)-xd)^2 + (actualPos(2)-yd)^2); 
    thetad = atan2((yd-actualPos(2)),(xd-actualPos(1)));
    thetae = (angulo(3)-thetad);

        if thetae > pi
            thetae = thetae - 2*pi;
        elseif thetae < -pi
            thetae = thetae + 2*pi;
        end
            
        w = -kpr*thetae;
        v = vmax*tanh(d*kpt/vmax);
        
        vr = v + (wheelbase*w)/2; %% Esto iría al PWM
        vl = v - (wheelbase*w)/2; %% Esto iría al PWM


    [~, position_pioneer]=sim.simxGetObjectPosition(clientID,pioneer_block,-1,...
        sim.simx_opmode_streaming);


    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor,vl,...
        sim.simx_opmode_blocking);
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor,vr,...
        sim.simx_opmode_blocking);

    
        figure(1)
        plot(xd,yd, '+');
        scatter(actualPos(1),actualPos(2));
         axis([-6 6 -6 6])
        hold on

        if d<0.1
            next_j=next_j +1;
            pause(1);
        end

        Thetae(i) = thetae;
        i = i+1;
        t = t + dt;

        [~,actualPos]=sim.simxGetObjectPosition(clientID, pioneer_block, ...
            -1, sim.simx_opmode_buffer);
        [~,angulo]=sim.simxGetObjectOrientation(clientID, pioneer_block, ...
            -1,sim.simx_opmode_buffer);
        
end

    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor,0,...
        sim.simx_opmode_blocking);
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor,0,...
        sim.simx_opmode_blocking);
    disp('Conexión con Coppelia Terminada');
    sim.simxFinish(clientID);
end
sim.delete(); % Llamar al destructor!
%This code uses a control law based on the hyperbolic tangent function to track a desired velocity and angular velocity. 
% The control law includes a feedback term to reduce tracking error. The robot motion is computed by integrating the kinematic 
% equations for a differential drive robot. The robot's motion is plotted in real time. You can modify the code to simulate different 
% robot behaviors and experiment with different control laws.s