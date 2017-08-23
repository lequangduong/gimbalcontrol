clc;clear;close all
load('variable.mat')
%% ============ SCHEME variable.mat ====================
% gimble 
%    - param
%          -- n            -> Number of DoF
%          -- stime        -> sample time
%          -- t            -> time
%          -- tf           -> time finite
%          -- theta        -> {theta_1; theta_2; theta_3]
%          -- q            -> [q_1; q_2; q_3]
%          -- Dq           -> [Dq_1; Dq_2; Dq_3]
%          -- DDq          -> [DDq_1; DDq_2; DDq_3]
%          star ~ reference 
%          -- us           -> [us_1; us_2; us_3]
%          -- qs           -> [qs_1; qs_2; qs_3]
%          -- Dqs          -> [Dqs_1; Dqs_2; Dqs_3]
%          -- DDqs         -> [DDqs_1; DDqs_2; DDqs_3]
%     - sym                -> symbolic 
%          -- kin          -> kinematics
%               --- R      -> R O0x0y0z0
%               --- Rf     -> R OXYZ (f)
%               --- T      -> T O0x0y0z0
%               --- Tf     -> T OXYZ (f)
%               --- o      -> omega O0x0y0z0
%               --- of     -> omega OXYZ (f)
%     - num                -> numeric
%          -- kin          -> kinematics
%               --- R      -> R O0x0y0z0
%               --- Rf     -> R OXYZ (f)
%               --- T      -> T O0x0y0z0
%               --- Tf     -> T OXYZ (f)
%               --- o      -> omega O0x0y0z0
%               --- of     -> omega OXYZ (f)
%     - sym                -> symbolic 
%          -- dyn          -> dynamics
%               --- M      -> M matrix
%               --- C      -> C matrix
%               --- G      -> G matrix
%     - num                -> numeric
%          -- dyn          -> dynamics
%               --- M      -> M matrix
%               --- fM     -> M handle function
%               --- C      -> C matrix
%               --- fC     -> C handle function
%               --- G      -> G matrix
%               --- fG     -> G handle function
% drone
%     - sym                -> symbolic 
%          -- pos          -> [X; Y; Z] position of drone
%% ============ INPUT drone and target =============
n = 3;
gimbal.param.n = n;
gimbal.param.stime = 0.01; 
gimbal.param.tf = 10;
gimbal.param.t = 0:gimbal.param.stime:gimbal.param.tf;

% ====== Quadcopter / Drone ==========
t = gimbal.param.t;
tf = gimbal.param.tf;
drone.num.pos = [0.02*t.^2; % X
                 0.2*t; % Y
                 0.2*sin(10*t/tf*pi-pi/2)+3 %Z
                 ];
drone.num.acc = [0.04*t; % X
                 0*t; % Y
                 -(10/tf*pi)^2*0.2*sin(10*t/tf*pi-pi/2)+2 %Z
                 ];              
drone.num.dir  = [pi/18*sin(t/tf*pi-pi/2); % ROLL
                  pi/18*sin(t/tf*pi-pi/2); % PITCH
                  pi/6*sin(t/tf*pi); % YAW Must begin at 0
                 ];
% ====== Target =======
obj.num.pos = [0.02*t.^2+0.5; % X
               0.2*t+0.6; % Y
               0*t %Z
               ];           

%% ===================== Controller =====================
% Input simulink
% reference vallue - star
qs = double(gimbal.invkin( obj, drone, gimbal, [0; -pi/2] ));
Dqs = [0 diff(qs(1,:))./diff(t); 0 diff(qs(2,:))./diff(t); 0 diff(qs(3,:))./diff(t)];
DDqs = [0 diff(Dqs(1,:))./diff(t); 0 diff(Dqs(2,:))./diff(t); 0 diff(Dqs(3,:))./diff(t)];
DDX = drone.num.acc(1,:); DDY = drone.num.acc(2,:); DDZ = drone.num.acc(3,:);
xs = [qs; Dqs];
us = gimbal.num.dyn.fL(DDX,DDY,DDZ,DDqs(1,:),DDqs(2,:),DDqs(3,:),Dqs(1,:),Dqs(2,:),Dqs(3,:),qs(1,:),qs(2,:),qs(3,:));

% initial value
q0 = zeros(3,1); Dq0 = zeros(3,1); DDq0 = zeros(3,1);
x0 = [q0; Dq0]; Dx0 = [Dq0; DDq0];

%% LQG - MPC Controller
controller.param.stime = gimbal.param.stime;
controller.param.Q0 = 1; 
controller.param.R0 = 1;

controller.param.wd = randn(2*n,1)*0;
controller.param.vd = randn(n,1)*0;
controller.ref.x = xs;
controller.ref.u = us;
controller.init.x = x0;
controller.init.Dx = Dx0;
% MPC constrain
controller.param.umax = 6; % kg*m/s^2 * m
controller.param.umin = -6;
controller.param.T = 10; % number of step discreted linear system
controller.param.Nc = 8; % number of step predicted (horizontal)
% Delay
controller.param.tDelay = 4; % ~ tDelay * stime/T
                              % ~ 4*0.01/10 = 0.004(s)
                              % tDelay must less than Nc

[u_lqr, y_lqr, x_lqr] = lqg_solve(controller,gimbal,drone);
[u_mpc, y_mpc, x_mpc] = mpc_solve(controller,gimbal,drone);
plot_output(gimbal.param.stime, tf, x_lqr, x_mpc, xs, u_lqr, u_mpc, us);

%% =================== Simulation ========================
T = 10;
ts = 0:gimbal.param.stime:tf;
t = 0:gimbal.param.stime/T:tf;
input.theta = [ts', xs(1:3,:)'];
%input.theta = [t', x_mpc(1:3,:)'];
input.targetposition = [ts', obj.num.pos'];
input.quadposition = [ts' drone.num.pos'];
input.quadrotation = [ts' (drone.num.dir)'];%RPY2XYZ
input.quadposition_line = drone.num.pos';
input.targetposition_line = obj.num.pos';
open Kinematics.slx
%open Dynamics.slx

%% =================== FUNCTION =============

function [u, y, xhat] = lqg_solve(controller,gimbal,drone)
    xs = controller.ref.x;
    us = controller.ref.u; 
    x0 = controller.init.x;
    Dx0 = controller.init.Dx;
    stime = controller.param.stime;
    Q0 = controller.param.Q0;
    R0 = controller.param.R0;
    wd = controller.param.wd;
    vd = controller.param.vd;
    T = controller.param.T;    
    tDelay = controller.param.tDelay;
    if(tDelay > T)
        error('tDelay time must less than T');
    end
    
    DDX = drone.num.acc(1,:); 
    DDY = drone.num.acc(2,:); 
    DDZ = drone.num.acc(3,:);
    n = size(xs,1)/2; %Number of DOF
    N = size(xs,2); %Number of points of linearization
    P = T*(N-1);
    % Example -> Cach chia discrete system
    % *.........*.........*........* (P = 30, N = 4, T = 10)
    % 0        0.01      0.02     0.03
    % *    .      .     .     .     .     .     .     .     .      *
    % 0  0.001  0.002 0.003 0.004 0.005 0.006 0.007 0.008 0.009 0.01
    % *    .      .     .     .     .     .     .     .     .      *
    %0.01  0.011  0.012 0.013 0.014 0.015 0.016 0.017 0.018 0.019 0.02
    
    Dxs = [zeros(2*n,1) diff(xs,1,2)./stime];
    % estimate states
    xhat = zeros(2*n, P);
    Dxhat = zeros(2*n, P);

    %model state
    xmod = zeros(2*n, P);
    Dxmod = zeros(2*n, P);

    %control input
    u = zeros(n, P);

    % initialization
    xhat(:,1) = x0;
    xmod(:,1) = x0;
    Dxhat(:,1) = Dx0;
    Dxmod(:,1) = Dx0;

    % % Make system
    C = gimbal.num.dyn.fC();
    D = gimbal.num.dyn.fD();

    y = C*xhat+D*u; % initial measured output
    ys = C*xs+D*us;

    Q = Q0*(C'*C);
    R = R0*eye(n);
    %qk = 200;
    qk = 1;
    Qk = [qk 0 0; % QN use for kalman
          0 qk 0;
          0 0 qk];
      
    for k = 1:(N-1)
        A = gimbal.num.dyn.fA(DDX(k),DDY(k),DDZ(k),xs(4,k),xs(5,k),xs(6,k),xs(1,k),xs(2,k),xs(3,k),us(1,k),us(2,k),us(3,k));
        B = gimbal.num.dyn.fB(xs(2,k),xs(3,k));
        sys = ss(A,B,C,D);
        K = lqr(sys,Q,R);
        %% LQG state-estimator
        % x_hat_dot = (A-LC-BK)x_hat + Ly
        % u = Kx -> u
        for i=1:T
            p = (k-1)*T+i; %     
            % Check delay output 
            % if tDelay = 3 (~0.003 second)
            % 1      2       3       4          (p)
            % *      .       .       .
            % 0     0.001   0.002   0.003       (second)
            % yDelay                yDelay
            if mod(p+tDelay-1,tDelay)==0 
                yDelay = y(:,p);
            end
            if tDelay ==0 
                yDelay = y(:,p);
            end
            %Linearization Equation of us, xs, Dxs
            usk_i= us(:,k) + (us(:,k+1) - us(:,k))/T * (i-1);
            xsk_i = xs(:,k) + (xs(:,k+1) - xs(:,k))/T * (i-1);
            ysk_i = ys(:,k) + (ys(:,k+1) - ys(:,k))/T * (i-1);
            Dxsk_iadd1 = Dxs(:,k) + (Dxs(:,k+1) - Dxs(:,k))/T *(i);              
            % LQG controller
            [~,L,~] = kalman(sys,Qk,R);
            u(:,p) = usk_i - K*xhat(:,p) + K*xsk_i;                             %y(:,p)
            Dxhat(:,p+1) = Dxsk_iadd1 + (A - L*C - B*K)*(xhat(:,p) - xsk_i) + L*(yDelay - ysk_i);
            xhat(:,p+1) = Dxhat(:,p+1)*(stime/T) + xhat(:,p);  
            % Gimbal model
            Dxmod(:,p+1) = gimbal.num.dyn.fxu(DDX(k),DDY(k),DDZ(k),...
                                              xmod(4,p),xmod(5,p),xmod(6,p),...
                                              xmod(1,p),xmod(2,p),xmod(3,p),...
                                              u(1,p),u(2,p),u(3,p));
            xmod(:,p+1) = Dxmod(:,p+1)*(stime/T) + xmod(:,p) + wd;  
            y(:,p+1) = C*xmod(:,p+1) + vd;     
        end
    end
end

function [u, y, xhat] = mpc_solve(controller,gimbal,drone)
    xs = controller.ref.x;
    us = controller.ref.u; 
    x0 = controller.init.x;
    Dx0 = controller.init.Dx;
    stime = controller.param.stime;
    Q0 = controller.param.Q0;
    R0 = controller.param.R0;
    wd = controller.param.wd;
    vd = controller.param.vd;    
    umax = controller.param.umax;
    umin = controller.param.umin;   
    T = controller.param.T;
    Nc = controller.param.Nc;
    if(Nc > T)
        error('Nc must less than T');
    end      
    tDelay = controller.param.tDelay;
    if(tDelay > Nc)
        error('tDelay time must less than Nc');
    end    
    
    DDX = drone.num.acc(1,:); 
    DDY = drone.num.acc(2,:); 
    DDZ = drone.num.acc(3,:);
    n = size(xs,1)/2; %Number of DOF
    N = size(xs,2); %Number of points of linearization
    P = T*(N-1);
    % Example -> Cach chia discrete system
    % *.........*.........*........* (P = 30, N = 4, T = 10)
    % 0        0.01      0.02     0.03
    % *    .      .     .     .     .     .     .     .     .      *
    % 0  0.001  0.002 0.003 0.004 0.005 0.006 0.007 0.008 0.009 0.01
    % *    .      .     .     .     .     .     .     .     .      *
    %0.01  0.011  0.012 0.013 0.014 0.015 0.016 0.017 0.018 0.019 0.02
    Dxs = [zeros(2*n,1) diff(xs,1,2)./stime];
    
    % estimate states
    xhat = zeros(2*n, P);
    Dxhat = zeros(2*n, P);

    %model state
    xmod = zeros(2*n, P);
    Dxmod = zeros(2*n, P);

    %control input
    u = zeros(n, P);

    % initialization
    xhat(:,1) = x0;
    xmod(:,1) = x0;
    Dxhat(:,1) = Dx0;
    Dxmod(:,1) = Dx0;

    % % Make system
    C = gimbal.num.dyn.fC();
    D = gimbal.num.dyn.fD();

    y = C*xhat+D*u; % initial measured output
    ys = C*xs+D*us;

    Q0 = Q0*(C'*C);
    R0 = R0*eye(n);
    %qk = 200;
    qk = 1;
    Qk = [qk 0 0; % QN use for kalman
          0 qk 0;
          0 0 qk];

    DeluPredicted = zeros(Nc,n);  
    for k = 1:(N-1)
        A = gimbal.num.dyn.fA(DDX(k),DDY(k),DDZ(k),xs(4,k),xs(5,k),xs(6,k),xs(1,k),xs(2,k),xs(3,k),us(1,k),us(2,k),us(3,k));
        B = gimbal.num.dyn.fB(xs(2,k),xs(3,k));
        sys = ss(A,B,C,D);
        sysd = c2d(sys,stime/T,'zoh');
        A = sysd.A; B = sysd.B;
%         w.Q = Q*sys.C'*sys.C;
%         w.R = R*eye(3);
%         w.Nc = Nc; % constraint checking horizon
        
        %% Discrete system
        %Ac*u(k) <= b0 + Bx*x(k)
        %input constraint umin < u < umax
        Delumax = zeros(n,1);Delumin = zeros(n,1);
        for i=1:3
            Delumax(i) = umax-max(us(i,k),us(i,k+1)); %just constant
            Delumin(i) = umin-min(us(i,k),us(i,k+1));   
        end
        R_kal = R0*eye(n);
        Ac = [eye(Nc*n); -eye(Nc*n)];
        %b0 = [];
        %b0 = [ones(Nc*n,1).*[umax;umax]; -ones(Nc*n,1).*[umin; umin]];
        b0 = repmat(Delumax,Nc,1);
        b0 = [b0; -repmat(Delumin,Nc,1)];
        %Bx = 0;
        Q = [];
        % Q using Lyapunov Equation
        K = -dlqr(A,B,Q0,R0);
        Q_ = dlyap((A+B*K)',Q0+K'*R0*K);
        for i=1:Nc-1
            Q = blkdiag(Q,Q0);
        end
        Q = blkdiag(Q,Q_);
        R = [];
        for i=1:Nc
            R = blkdiag(R,R0);
        end
        %M_ = [];
        M_ = zeros((2*n)*Nc, 2*n);
        for i=1:Nc
            %M_ = [M_; A^i];
            rol = (i-1)*2*n+1:i*2*n; % rol change
            M_(rol, 1:2*n) = A^i;
        end
        %C_ = [];
        C_ = convolution(A, B, n, Nc);
        H = C_'*Q*C_ + R;
        F = C_'*Q*M_;
        %G = M_'*Q*M_+Q0;

%         % unconstrain
%         % u(k) = -[1 0 0 0]*H^-1*F*x(k) = K_N*x(k)
%         K_mpc = -H^-1*F;
%         K_mpc = K_mpc(1:n,:);
%         Delx = xhat(:,k)-xs(:,k);
%         Delu = K_mpc*Delx;
        for i=1:T
            p = (k-1)*T+i; %  
            % Check delay output 
            % if tDelay = 3 (~0.003 second)
            % 1      2       3       4          (p)
            % *      .       .       .
            % 0     0.001   0.002   0.003       (second)
            % yDelay                yDelay
            if mod(p+tDelay-1,tDelay)==0 
                yDelay = y(:,p);
            end  
            if tDelay ==0 
                yDelay = y(:,p);
            end            
            %Linearization Equation of us, xs, Dxs
            usk_i= us(:,k) + (us(:,k+1) - us(:,k))/T * (i-1);
            xsk_i = xs(:,k) + (xs(:,k+1) - xs(:,k))/T * (i-1);
            ysk_i = ys(:,k) + (ys(:,k+1) - ys(:,k))/T * (i-1);
            Dxsk_iadd1 = Dxs(:,k) + (Dxs(:,k+1) - Dxs(:,k))/T *(i);   
            % Constrain  
            %uPredicted = quadprog(H,F*x(:,i),Ac,b0+Bx*x(:,i));
            Delx = xhat(:,p) - xsk_i;
            options = optimoptions('quadprog','Display','off');        
            uPredicted = quadprog(H,F*Delx,Ac,b0,[],[],[],[],[],options); %Bx = 0
            % Turn of the warning of quadprog function
            [~,warningID] = lastwarn;
            warning('off',warningID);
            if isempty(uPredicted)
                if  size(DeluPredicted,1) > 1
                    DeluPredicted = DeluPredicted(2:end,:);
                end
            else     
                for j=1:Nc
                    DeluPredicted(j,:)= uPredicted(n*(j-1)+1:n*j)';
                end   
            end
            Delu = DeluPredicted(1,:)';
            % Estimation
            K = lqr(sys,Q0,R0);
            A = sys.A; B = sys.B; C = sys.C;
            [~,L,~] = kalman(sys,Qk,R_kal);
            u(:,p) = usk_i + Delu; % check sign                                  y(:,p)
            Dxhat(:,p+1) = Dxsk_iadd1 + (A - L*C - B*K)*(xhat(:,p) - xsk_i) + L*(yDelay - ysk_i);
            xhat(:,p+1) = Dxhat(:,p+1)*(stime/T) + xhat(:,p);  
            % Gimbal model
            Dxmod(:,p+1) = gimbal.num.dyn.fxu(DDX(k),DDY(k),DDZ(k),...
                                              xmod(4,p),xmod(5,p),xmod(6,p),...
                                              xmod(1,p),xmod(2,p),xmod(3,p),...
                                              u(1,p),u(2,p),u(3,p));
            xmod(:,p+1) = Dxmod(:,p+1)*(stime/T) + xmod(:,p) + wd;  
            y(:,p+1) = C*xmod(:,p+1) + vd;     
        end
    end
end

function C_ = convolution(A, B, n, Nc)
%C_=[];
C_ = zeros(2*n*Nc, n*Nc);
for j=1:Nc
%   col_i = [];
    col_j = zeros(2*n*Nc, n);
    for i=1:Nc
        rol = (i-1)*2*n+1:i*2*n; % rol change
        if j>i
            %col_i = [col_i; zeros(size(B))];
            col_j(rol, 1:n) = zeros(size(B));
        else
            %col_i = [col_i; A^(i-j)*B];
            col_j(rol, 1:n) = A^(i-j)*B;
        end
    end
    rol = 1:2*n*Nc; % rol change
    col = (j-1)*n+1  :j*n;
    C_(rol, col) = col_j;
%     C_ = [C_ col_j];
end

end

function plot_output(stime, tf, x_lqr, x_mpc, xs, u_lqr, u_mpc, us)
T = 10; n = 3;
t = 0:stime/T:tf;
ts = 0:stime:tf;

% plot x
for p=1:2*n
    figure(p)
    subplot(3,2,1)
    plot(ts,xs(p,:),t,x_lqr(p,:))
    legend(['x^*(',char(p+48),')'],['x(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('rad')
    title(['Response of x_',char(p+48),' with lqr'])

    subplot(3,2,2)
    plot(ts,xs(p,:),t,x_mpc(p,:))
    legend(['x^*(',char(p+48),')'],['x(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('rad')
    title(['Response of x_',char(p+48),' with mpc'])
      
    % zoom in t = 0 -> 0.5 (s)
    subplot(3,2,3)
    plot(ts(1:50),xs(p,1:50),t(1:500),x_lqr(p,1:500))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of x_',char(p+48),' with lqr',' (zoom t < 0.5s)'])

    subplot(3,2,4)
    plot(ts(1:50),xs(p,1:50),t(1:500),x_mpc(p,1:500))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of x_',char(p+48),' with mpc',' (zoom t < 0.5s)'])   
    
    % zoom in t = 0.5 -> 10 (s)
    subplot(3,2,5)
    plot(ts(51:end),xs(p,51:end),t(501:end),x_lqr(p,501:end))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of x_',char(p+48),' with lqr',' (zoom t > 0.5s)'])

    subplot(3,2,6)
    plot(ts(51:end),xs(p,51:end),t(501:end),x_mpc(p,501:end))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of x_',char(p+48),' with mpc',' (zoom t > 0.5s)'])     
end
% plot u
for p=1:n
    figure(p+2*n)
    subplot(3,2,1)
    plot(ts,us(p,:),t(1:end-1),u_lqr(p,:))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with lqr'])

    subplot(3,2,2)
    plot(ts,us(p,:),t(1:end-1),u_mpc(p,:))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with mpc'])
    
    % zoom in t = 0 -> 0.5 (s)
    subplot(3,2,3)
    plot(ts(1:50),us(p,1:50),t(1:500),u_lqr(p,1:500))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with lqr',' (zoom t < 0.5s)'])

    subplot(3,2,4)
    plot(ts(1:50),us(p,1:50),t(1:500),u_mpc(p,1:500))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with mpc',' (zoom t < 0.5s)'])  
    
    % zoom in t = 0.5 -> 10 (s)
    subplot(3,2,5)
    plot(ts(51:end),us(p,51:end),t(501:end-1),u_lqr(p,501:end))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with lqr',' (zoom t > 0.5s)'])

    subplot(3,2,6)
    plot(ts(51:end),us(p,51:end),t(501:end-1),u_mpc(p,501:end))
    legend(['u^*(',char(p+48),')'],['u(',char(p+48),')'])
    xlabel('t (s)')
    ylabel('Nm')
    title(['Response of u_',char(p+48),' with mpc',' (zoom t > 0.5s)'])    
end

end