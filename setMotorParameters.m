function motor = setMotorParameters(motorType)
%SETMOTORPARAMETERS Paramètres du moteur synchro-réluctant (SynRel).
%   motor = SETMOTORPARAMETERS("SynRelLaplace") renvoie une structure
%   contenant les paramètres électriques et mécaniques du moteur SynRel
%   du laboratoire Laplace, repris de
%   sources/v20240513_CartoComplete/03_Simulink/SetMotorParameters.m.
%
%   Les cartographies (directe, inverse, trajectoire de courants) ne sont
%   pas chargées ici : elles sont ajoutées à la structure par
%   simulink/initSimulink.m à partir des fichiers du dossier data/.
%
%   Champs de la structure motor :
%     model, type     - identification
%     p               - nombre de paires de pôles
%     R               - résistance statorique [Ohm]
%     Ld, Lq, Ldq     - inductances nominales dq [H] (découplage seulement)
%     Phif            - flux d'aimant [Wb] (nul pour un SynRel)
%     J, F, C0        - inertie [kg.m^2], frottement visqueux [N.m.s],
%                       frottement sec [N.m]
%     iMax, vMax      - limites courant [A] et tension [V]
%     omMax           - vitesse maximale [rad/s]
%     thetaPeriod     - période mécanique des cartographies dq [rad]
%
%   See also loadFemmRawData, buildFluxMap, invertFluxMap.

arguments
    motorType (1,1) string {mustBeMember(motorType, "SynRelLaplace")} ...
        = "SynRelLaplace"
end

motor = struct;

switch motorType
    case "SynRelLaplace"
        motor.model = 'SynREL_Laplace';
        motor.type  = 'SynRel';
        motor.sn    = '001';
        motor.p     = 2;            % paires de pôles
        % Résistance du banc HBK (Colab_Andrieu) : valeur utilisée pour
        % calculer le flux des données expérimentales, retrouvée par
        % régression sur ces données. Ancienne valeur : 0.5170 Ohm.
        motor.R     = 0.65;         % Ohm  - résistance statorique
        motor.Ld    = 0.0367;       % H    - inductance d nominale (découplage)
        motor.Lq    = 0.0163;       % H    - inductance q nominale (découplage)
        motor.Ldq   = diag([motor.Ld, motor.Lq]);
        motor.Phif  = 0;            % Wb   - pas d'aimant
        motor.J     = 10e-3;        % kg.m^2 - inertie
        % Frottement visqueux : SetMotorParameters.m donne 6e-5 mais le
        % modèle synRel_simu_LUT.slx applique un gain 100*motor.F ; on
        % retient la valeur effective (identique à SetSynRelMotorParameters.m).
        motor.F     = 6e-3;         % N.m.s - frottement visqueux
        motor.C0    = 0;            % N.m   - frottement sec
        motor.iMax  = 10;           % A
        motor.vMax  = 250;          % V
        motor.omMax = 500;          % rad/s
end

% Les cartographies dq d'un SynRel à p paires de pôles issues de FEMM sont
% périodiques d'un quart de tour électrique (symétrie des axes d/q) :
% période mécanique = 2*pi / (4*p).
motor.thetaPeriod = pi / (2 * motor.p);

end
