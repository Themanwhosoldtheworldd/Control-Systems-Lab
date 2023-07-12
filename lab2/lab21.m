%a = arduino;
% Set the desired position
des_pos = 5;

%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND
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

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %

writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

positionData = [];
velocityData = [];
uData = [];
timeData = [];

t=0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

% WAIT A KEY TO PROCEED
disp(['Connect cable from Arduino to Input Power Amplifier and then press enter to start controller']);
pause()


%START CLOCK
tic
 
 
while(t<5)  
    
position = readVoltage(a, 'A5'); % position
velocity = readVoltage(a,'A3'); % velocity
x2 = 3 * Vref_arduino * position / 5;

x1 = (2 * (2 * velocity * Vref_arduino / 5 - V_7805))/KT;

e = des_pos - x2;

if abs(e) > 10
    e = sign(e) * 10;
end

%Define the Control Signal
u = -K1 * x1 - K2 * x2 + Kr * des_pos;

if abs(u) > 10
    u = sign(u) * 10;
end

%Send the Control Signal
    if u > 0
        writePWMVoltage(a, 'D6', 0)
        writePWMVoltage(a, 'D9', min(round(u/2*5/Vref_arduino),5))
    else
        writePWMVoltage(a, 'D9', 0)
        writePWMVoltage(a, 'D6', min(round(-u/2*5/Vref_arduino),5))
    end

t=toc;
    
timeData = [timeData t];
positionData = [positionData x2];
velocityData = [velocityData x1];
uData = [uData u];
%desposdata = [despos des_pos];
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
plot(timeData,uData);
title('controller')

%figure
%plot(tmeData,desposdata);
%title('DesPos')
disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();
