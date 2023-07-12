%a = arduino;
% Set the desired position
des_pos = 6;

%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND
V_7805=5.4787;
Vref_arduino=5.1032;

%Values Calculated in the System Identification
Kmi = 1/36;
KT = 3.887*10^(-3);
Km = 224.08;
Tm = 520*10^(-3);
K0 = 0.229;

%Trial and Error for K1, K2, Ki
%K1 = 0.01;
%K2 = 2.055;
%Ki = 3.1178;

K1 = 0.007;
K2 = 2.5;
Ki = 3.5;

%Check For Stability
if(Ki > 0 && Ki < K2*(1+K1*Km)/Tm && K1 > -1/Km)
    disp("The System Is Stable given those Gains");
    pause();
else
    disp("The System Is Unstable, it will Most Likely Explode. You have been Warned!!!!");
    pause();
end



% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

positionData = [];
velocityData = [];
eData = [];
timeData = [];
zData = [];
controlData = [];
t=0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

% WAIT A KEY TO PROCEED
disp(['Connect cable from Arduino to Input Power Amplifier and then press enter to start controller']);
pause()


%START CLOCK
tic

% Initialize previous position and time
prev_position = readVoltage(a, 'A5');
prev_time = 0;
prev_z_dot = 0;

% Assign initial value to z
z = 0;

while (t < 7)
    position = readVoltage(a, 'A5'); % position
    velocity = readVoltage(a, 'A3'); % velocity
    x2 = 3 * Vref_arduino * position / 5;
    x1 = (2 * (2 * velocity * Vref_arduino / 5 - V_7805)) / KT;

    % Calculate the derivative of z (z_dot)
    z_dot = x2 - des_pos;
    
    % Integrate z_dot
    delta_t = toc - t;
    z = z + (delta_t)*(z_dot);

    % Define the Control Signal with dynamic feedback
    u = -K1 * x1 - K2 * x2 - Ki * z;
    
     if abs(u) > 10
        u = sign(u) * 10;
     end

    % Send the Control Signal
    if u > 0
        writePWMVoltage(a, 'D6', 0);
        writePWMVoltage(a, 'D9', min(round(u / 2 * 5 / Vref_arduino), 5));
    else
        writePWMVoltage(a, 'D9', 0);
        writePWMVoltage(a, 'D6', min(round(-u / 2 * 5 / Vref_arduino), 5));
    end

    t = toc;

    % Update previous position, time, and z_dot
    prev_position = position;
    prev_time = t;
    prev_z_dot = z_dot;

timeData = [timeData t];
positionData = [positionData x2];
velocityData = [velocityData x1];
zData = [zData z];
controlData = [controlData u];
end

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %

writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

disp(['End of control Loop. Press enter to see diagramms']);
pause();


figure
plot(timeData,positionData);
title('position')

figure
plot(timeData,velocityData);
title('velocity')

figure
plot(timeData, zData);
title ('z')

figure
plot(timeData,controlData);
title('Controller')


disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();


        

