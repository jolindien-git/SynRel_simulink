function K = polePlacement2ndOrder(zeta,wn)
    % polePlacement2ndOrder function
    %
    % Desired Carateristc Equation
    %  1         2zeta
    % ---- p^2+ ------- p + 1
    % wn^2        wn
    %
    % overshoot 5%, Trbf x wnbf = 3
    %
    % zeta  : Damping factor
    % wn    : natural frequency
    % tau   : first order time constant
    % k     : static gain

    Desiredpoles = roots([1/wn^2 2*zeta/wn 1]);
    
    A = [
            0 0;
            -1 0
        ];
    B = [
            1
            0    
        ];

    K = acker(A,B,Desiredpoles);
end
