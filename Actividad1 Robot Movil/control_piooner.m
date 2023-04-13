%% Inicio de conexion
sim=remApi('remoteApi'); % usando el prototipo de función (remoteApiProto.m)
sim.simxFinish(-1); % Cerrar las conexiones anteriores en caso de que exista una
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Verificación de la conexión
if (clientID>-1)

    disp('Conexión con Coppelia iniciada');
    
    %%% Codigo de control
    %% Preparación
    % Se crean los Handles correspondientes al motor, sensor ultrasonico,
    % el robot y la referencia
    [~, left_motor]=sim.simxGetObjectHandle(clientID,...
    'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [~, right_motor]=sim.simxGetObjectHandle(clientID,...
    'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [~, front_sensor]=sim.simxGetObjectHandle(clientID,...
    'Pioneer_p3dx_ultrasonicSensor4',sim.simx_opmode_blocking);
    [~, pioneer_block]=sim.simxGetObjectHandle(clientID,...
    'Pioneer_p3dx',sim.simx_opmode_blocking);
    [~, target_block]=sim.simxGetObjectHandle(clientID,...
    'Cuboid0',sim.simx_opmode_blocking);
    %Constante
    [~, position_leftWheel]=sim.simxGetObjectPosition(clientID,pioneer_block,-1,...
    sim.simx_opmode_streaming);

    %% Acciones
    % Se prepara la velocidad del motor izquierdo y se comienza a medir el
    % sensor ultrasonico
    [~] = sim.simxSetJointTargetVelocity(clientID,left_motor,0.5,...
    sim.simx_opmode_blocking);
    [~,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,...
    front_sensor, sim.simx_opmode_streaming);

    % Se obtiene la posición del robot y de la referencia, tomando en
    % cuenta que al igual que el sensor infrarrojo, la primera medición se
    % realiza con simx_opmode_streaming y las siguientes con la funcion de
    % simx_opmode_buffer
    [~, position_pioneer]=sim.simxGetObjectPosition(clientID,pioneer_block,-1,...
    sim.simx_opmode_streaming);
    [~, target]=sim.simxGetObjectPosition(clientID,target_block,-1,...
    sim.simx_opmode_streaming);
    for i = 1:200
        % Se mide el sensor infrarrojo
        [~,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,...
        front_sensor, sim.simx_opmode_buffer);
        y_info = sprintf('La medicion del sensor ultrasonico es %f\n',norm(detectedPoint));
        disp(y_info)
        % Se mide la posición de cada objeto y se muestra la resta de ambos
        % en la pantalla junto con el valor detectado por el sensor

        % infrarrojo
        [~, position_pioneer]=sim.simxGetObjectPosition(clientID,pioneer_block,-1,...
        sim.simx_opmode_buffer);
        [~, target]=sim.simxGetObjectPosition(clientID,target_block,-1,...
        sim.simx_opmode_buffer);
        x_info = sprintf('La diferencia entre ambos objetos es (%f, %f)\n'...
        , position_pioneer(1)-target(1), position_pioneer(2)-target(2));
        disp(x_info)
        pause(0.1)
    end
    % Se detiene el robot dejando nuevamente la velocidad del motor en 0
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor,0,...
    sim.simx_opmode_blocking);
    disp('Conexión con Coppelia Terminada');
    sim.simxFinish(clientID);
end
sim.delete(); % Llamar al destructor!