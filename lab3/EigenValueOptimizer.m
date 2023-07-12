Kmi = 1/36;
KT = 3.887*10^(-3);
Km = 224.08;
Tm = 520*10^(-3);
K0 = 0.229;

% Objective Function
objFunc = @(x) prod(abs(eig([(-1-x(1)*Km)/Tm -x(2)*Km/Tm -x(3)*Km/Tm; Kmi*K0 0 0; 0 1 0])));

% Nonlinear inequality constraints
nonlcon = @(x) deal([], [-x(1) - 1/Km; -x(3); -real(eig([(-1-x(1)*Km)/Tm, -x(2)*Km/Tm, -x(3)*Km/Tm; Kmi*K0, 0, 0; 0, 1, 0]))-2]);

% Additional constraints to encourage real eigenvalues
realEigenConstraint = @(x) sum(imag(eig([(-1-x(1)*Km)/Tm, -x(2)*Km/Tm, -x(3)*Km/Tm; Kmi*K0, 0, 0; 0, 1, 0])));

% Combined constraint function
combinedConstr = @(x)deal(nonlcon(x), realEigenConstraint(x));


% Lower and Upper Bounds for the Variables
lb = [0.0001, 0.0001, 0.0001];
ub = [5, 5, 5];


% Initial guess for the variables
x0 = [0.1, 0.1, 0.1];

% Set the options for the optimization algorithm
options = optimoptions('fmincon', 'Display', 'iter');

% Perform the optimization
[x_opt, fval] = fmincon(objFunc, x0, [], [], [], [], lb, ub, nonlcon, options);

% Display the optimal values
disp("Optimal Values:");
disp("K1: " + x_opt(1));
disp("K2: " + x_opt(2));
disp("Ki: " + x_opt(3));
disp("Min Eigenvalue: " + (fval));


