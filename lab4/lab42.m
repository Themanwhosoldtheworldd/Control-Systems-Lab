%a = arduino;
% Set the desired position
des_pos = 5;
%The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND
V_7805=5.4787;
Vref_arduino=5.1032;

%Values Calculated in the System Identification
Kmi = 1/36;
KT = 3.887*10^(-3);
Km = 224.08;
Tm = 520*10^(-3);
K0 = 0.229;

%Trial and Error for K1
K1 = 0.0178;

%Calculation of K2 using K1
K2 = (K1^2*Km^2 + 2*K1*Km + 1)/(4*Kmi*K0*Km*Tm);
Kr = K2;

%Matrix A
A = [-1/Tm 0; Kmi*K0 0];

%Matrix B
B = [Km/Tm; 0];

%Matrix C
C = [0 1];

%Matrix L
p1 = sol.P1;  %sol.P1 and sol.P2 are produced when you run ObserverGain with the desired EigenValues
p2 = sol.P2;
L = [-p1/(Tm*Kmi*K0)+1/(Tm^2*Kmi*K0)+p2/(Kmi*K0); +p1 - 1/Tm];

%Matrix Ahat
Ahat = A - L*C;

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

positionData = [];
velocityData = [];
estimatedPositionData = [];
estimatedVelocityData = [];
timeData = [];
controlData = [];
t=0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

% WAIT A KEY TO PROCEED
disp(['Connect cable from Arduino to Input Power Amplifier and then press enter to start controller']);
pause()


%START CLOCK
tic

% Assign initial value to xhat and the control signal
xhat = [0; 0];
u = 0;

while (t < 7)
    position = readVoltage(a, 'A5'); % position
    velocity = readVoltage(a, 'A3'); % velocity
    
    x2 = 3 * Vref_arduino * position / 5;
    x1 = (2 * (2 * velocity * Vref_arduino / 5 - V_7805)) / KT;
   
    % Calculate the derivative of xhat (xhatdot)
    xhatdot = Ahat*xhat + L*C*[1; x2] + B*u;
    
    % Update the observer states
    delta_t = toc - t;
    xhat = xhat + xhatdot * delta_t;
    
    % Define the Control Signal 
    u = [-K1 -K2]*[xhat(1);xhat(2)] + Kr*des_pos;
    controlSignal = -K1 * xhat(1) - K2 * xhat(2) + Kr * des_pos;  %u and ControlSignal are the same thing
    controlSignal = double(controlSignal);

    if abs(controlSignal) > 10
        controlSignal = sign(controlSignal) * 10;
     end

    % Send the Control Signal
    if controlSignal > 0
        writePWMVoltage(a, 'D6', 0);
        writePWMVoltage(a, 'D9', min(round(controlSignal / 2 * 5 / Vref_arduino), 5));
    else
        writePWMVoltage(a, 'D9', 0);
        writePWMVoltage(a, 'D6', min(round(-controlSignal / 2 * 5 / Vref_arduino), 5));
    end

    t = toc;


timeData = [timeData t];
positionData = [positionData x2];
velocityData = [velocityData x1];
controlData = [controlData controlSignal];
estimatedVelocityData = [estimatedVelocityData xhat(1)];
estimatedPositionData = [estimatedPositionData xhat(2)];
end

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %

writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

disp(['End of control Loop. Press enter to see diagramms']);
pause();

figure
plot(timeData, positionData, 'b', 'LineWidth', 1.5); % Plot actual position in blue
hold on
plot(timeData, estimatedPositionData, 'r--', 'LineWidth', 1.5); % Plot estimated position in red dashed line
hold off
title('Position Comparison');
xlabel('Time');
ylabel('Position');
legend('Actual Position', 'Estimated Position');

figure
plot(timeData, velocityData, 'b', 'LineWidth', 1.5); % Plot actual position in blue
hold on
plot(timeData, estimatedVelocityData, 'r--', 'LineWidth', 1.5); % Plot estimated position in red dashed line
hold off
title('Velocity Comparison');
xlabel('Time');
ylabel('Velocity');
legend('Actual Velocity', 'Estimated Velocity');

figure
plot(timeData,controlData);
title('Controller')


disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();