Kmi = 1/36;
KT = 3.887*10^(-3);
Km = 224.08;
Tm = 520*10^(-3);
K0 = 0.229;


A = [-1/Tm 0; Kmi*K0 0];
eigA = eig(A);
C = [0 1];

%desiredEigenvalues = [-12 -8]; LAB42
desiredEigenvalues = [-12 -8];
%desiredEigenvalues = [-15 -8];%LAB41

syms P1 P2

L = [-P1/(Tm*Kmi*K0)+1/(Tm^2*Kmi*K0)+P2/(Kmi*K0); +P1 - 1/Tm];
obs = A - L*C;
% Calculate the characteristic polynomial
characteristic_poly = charpoly(obs);

% Display the characteristic polynomial
disp('Characteristic Polynomial:');
disp(characteristic_poly);

S = -characteristic_poly(2)/characteristic_poly(1);
P = characteristic_poly(3)/characteristic_poly(1);

eqn1 = S == desiredEigenvalues(1) + desiredEigenvalues(2);
eqn2 = P == desiredEigenvalues(1) * desiredEigenvalues(2);

sol = solve([eqn1, eqn2], P1, P2);

P1 = double(sol.P1);
P2 = double(sol.P2);

disp(['P1: ', num2str(P1)]);
disp(['P2: ', num2str(P2)]);
