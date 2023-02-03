function xhat = true_states(uu)

% fake state estimation for mavsim
%   - this function will be replaced with a state estimator in a later
%   chapter.

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = -uu(3+NN); % altitude
    u        = uu(4+NN);  % inertial velocity along body x-axis
    v        = uu(5+NN);  % inertial velocity along body y-axis
    w        = uu(6+NN);  % inertial velocity along body z-axis
    e0       = uu(7+NN);  % e0
    e1       = uu(8+NN);  % e1
    e2       = uu(9+NN);  % e2
    e3       = uu(10+NN); % e3
    p        = uu(11+NN); % body frame roll rate
    q        = uu(12+NN); % body frame pitch rate
    r        = uu(13+NN); % body frame yaw rate
    NN = NN+13;
    Va       = uu(1+NN);  % airspeed
    alpha    = uu(2+NN);  % angle of attack
    beta     = uu(3+NN);  % sideslip angle
    wn       = uu(4+NN);  % wind North
    we       = uu(5+NN);  % wind East
    wd       = uu(6+NN);  % wind down
    NN = NN+6;
    t        = uu(1+NN);   % time

    
    eulerAngles = Quaternion2Euler([e0; e1; e2; e3]);
    phi      = eulerAngles(1);       % roll angle         
    theta    = eulerAngles(2);       % pitch angle     
    psi      = eulerAngles(3);       % yaw angle 


    
    % estimate states (using real state data)
    pnhat    = pn;
    pehat    = pe;
    hhat     = h;
    Vahat    = Va;
    alphahat = alpha;
    betahat  = beta;
    phihat   = phi;
    thetahat = theta;
    chihat   = atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
    phat     = p;
    qhat     = q;
    rhat     = r;
    Vghat    = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
    wnhat    = wn;
    wehat    = we;
    psihat   = psi;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
    
end 