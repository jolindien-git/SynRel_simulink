% motorData = load('synRelSimuLutData.mat').motor;
trajectory = load('synRelSimuLutData.mat').trajectory;

%% map id = map(theta, torque)
% for, e.g, maintain a constant torque (tau)
tauRef = 2;
theta = linspace(0, pi/4);

[ThetaGrid, TauGrid] = meshgrid(trajectory.theta, trajectory.tauRef);
id_interp = interp2(ThetaGrid, TauGrid, trajectory.id, theta, tauRef, 'linear');

figure; 
plot(theta, id_interp)
legend("i_d")
xlabel("\theta")