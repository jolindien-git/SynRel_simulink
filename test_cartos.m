motorData = load('synRelSimuLutData.mat').motor;
trajectory = load('synRelSimuLutData.mat').trajectory;

%% id = map(theta, torque)
tauRef = 2;
theta = linspace(0, pi/4);

[ThetaGrid, TauGrid] = meshgrid(trajectory.theta, trajectory.tauRef);
id_interp = interp2(ThetaGrid, TauGrid, trajectory.id, theta, tauRef, 'linear');

figure; 
plot(theta, id_interp)
legend("i_d")
xlabel("\theta")

%% Linc
id = 6/2;
iq = 6/2;
theta = linspace(0, pi/4);

[ThetaGrid, IdGrid, IqGrid] = ndgrid(motorData.lutLinc.theta, motorData.lutLinc.id, motorData.lutLinc.iq);
interp3 = @(map) interpn(ThetaGrid, IdGrid, IqGrid, map, theta, id, iq, 'linear');
Ldd_interp = interp3(motorData.lutLinc.Ldd);
Ldq_interp = interp3(motorData.lutLinc.Ldq);
Lqd_interp = interp3(motorData.lutLinc.Lqd);
Lqq_interp = interp3(motorData.lutLinc.Lqq);

figure
plot(theta, Ldd_interp, theta, Ldq_interp, theta, Lqq_interp, theta, Lqd_interp)
legend('Ldd', 'Ldq', 'Lqq', 'Lqd')