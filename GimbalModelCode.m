%% ===============  Declaration =================
clc;clear;close all

% ========= Notation of all joint angles is theta 1, 2, 3
gimbal.param.n = 3;
gimbal.param.theta = sym('theta_',[gimbal.param.n 1],'real');
gimbal.param.u   = sym('u_'  ,[gimbal.param.n 1],'real');
% === q = theta; %This variable is equivalent
gimbal.param.q   = sym('q_'  ,[gimbal.param.n 1],'real');
gimbal.param.Dq  = sym('Dq_' ,[gimbal.param.n 1],'real');
gimbal.param.DDq = sym('DDq_',[gimbal.param.n 1],'real');
% === reference parameter
gimbal.param.us   = sym('us_'  ,[gimbal.param.n 1],'real');
gimbal.param.qs   = sym('qs_'  ,[gimbal.param.n 1],'real');
gimbal.param.Dqs  = sym('Dqs_' ,[gimbal.param.n 1],'real');
gimbal.param.DDqs = sym('DDqs_',[gimbal.param.n 1],'real');
% ========================
drone.sym.pos = [sym('X','real'); sym('Y','real'); sym('Z','real')];
drone.sym.acc = [sym('DDX','real'); sym('DDY','real'); sym('DDZ','real')]; % acceleration
drone.sym.dir = [sym('phi','real'); sym('theta','real'); sym('psi','real')]; %roll - pitch - yaw
obj.sym.pos = [sym('x','real'); sym('y','real'); sym('z','real')];

%% ===============  Read data of Gimbal motor in Exel Table ==================%%
data = getData(gimbal.param.n); 

%% ===================== Calculation of Transformation matrix =============== %
gimbal.sym.kin = forwardkinematics(data.sym,gimbal.param,drone.sym);
gimbal.num.kin = forwardkinematics(data.num,gimbal.param,drone.sym);
% Not use drone.num because rot function didn't catch matrix of time
gimbal.invkin = @inversekinematics;

gimbal.sym.dyn = robotdynamics(data.sym,gimbal.param,gimbal.sym.kin,drone.sym);
gimbal.num.dyn = robotdynamics(data.num,gimbal.param,gimbal.num.kin,drone.sym);


save('variable.mat')
%% ======== FUNCTION and CLASS ============%%

function gimbal_dynamics = robotdynamics(data,gimbal_param,gimbal_kin,drone_sym)
% The forward dynamics problem was constructed all with base frame
% coordinate system
% 
    n = gimbal_param.n; 
    q = gimbal_param.q; Dq = gimbal_param.Dq; DDq = gimbal_param.DDq;
    m = data.m; g = data.g;
    R = gimbal_kin.R; 
    T = gimbal_kin.T;
    omega = gimbal_kin.o;
    rC = cell(1,n);
    I = cell(1,n);
    for i=1:n
        rC_i = T{i} * [data.rC{i};1];
        rC{i} = rC_i(1:3);
        I{i} = data.I{i};
    end
    % Jacobi matrix
    J_T = cell(i,n);
    J_R = cell(1,n);
    for i = 1:n
        J_T{i} = jacobian(rC{i},q);
        J_R{i} = jacobian(omega{i},Dq);
    end
    
    M = zeros(n);
    for i = 1:n
        M = M + m(i)*J_T{i}'*J_T{i} + J_R{i}'*R{i}*I{i}*R{i}'*J_R{i};
    end
    gimbal_dynamics.M = M;
    
    In = eye(n);
    C = diffmat(M,q)*kron(In,Dq) - 1/2*(diffmat(M,q)*kron(Dq,In))';
    gimbal_dynamics.C = C;
    
    PI = 0;
    for i = 1:n
        PI = PI - m(i)*g'*rC{i};
    end
    gimbal_dynamics.PI = PI;
    G = jacobian(PI,q)';    
    gimbal_dynamics.G = G;
    
    if isa(data.l,'double')
        % Handle function
        %gimbal_dynamics.fM = matlabFunction(M);
        %gimbal_dynamics.fC = matlabFunction(C);
        %gimbal_dynamics.fG = matlabFunction(G);
    
        %=======================
        W = 0;
        for i=1:n
            F = -m(i).*drone_sym.acc;
            W = W + F'*rC{i};
        end
        QF = simplify(jacobian(W,q))';
        gimbal_dynamics.QF = QF;
        %gimbal_dynamics.fQF = matlabFunction(QF);

        u = gimbal_param.u;
        Qu = u;
        Q = Qu+QF;

        % === Get approximate vallue ====
        M = vpa(simplify(M),5);
        C = vpa(simplify(C),5);
        G = vpa(simplify(G),5);
        Q = vpa(simplify(Q),5);
        
        L = M*DDq + C*Dq + G - QF; % = u
        gimbal_dynamics.L = L;
        L = vpa(L,5);
        gimbal_dynamics.fL = matlabFunction(L);
        
        Fxu = [Dq; M^-1*(C*Dq - G + Q)];
        gimbal_dynamics.Fxu = Fxu;
        gimbal_dynamics.fxu = matlabFunction(Fxu);
        Fxu = vpa(Fxu,5);
        Hxu = q;
        gimbal_dynamics.Hxu = Hxu;
        
        % ==== Linearization ======
        x = [q;Dq];
        xs = [gimbal_param.qs;gimbal_param.Dqs];
        us = gimbal_param.us;
        % ------------
        gimbal_dynamics.A = subs(jacobian(Fxu,x),[x;u],[xs;us]);
        A = vpa(gimbal_dynamics.A,5);
        gimbal_dynamics.fA = matlabFunction(A);
        % ------------
        gimbal_dynamics.B = subs(jacobian(Fxu,u),[x;u],[xs;us]);
        gimbal_dynamics.fB = matlabFunction(gimbal_dynamics.B);
        % ------------
        gimbal_dynamics.C = subs(jacobian(Hxu,x),[x;u],[xs;us]);
        gimbal_dynamics.fC = matlabFunction(gimbal_dynamics.C);
        % ------------
        gimbal_dynamics.D = subs(jacobian(Hxu,u),[x;u],[xs;us]);
        gimbal_dynamics.fD = matlabFunction(gimbal_dynamics.D);      
        
    end
    
end

function theta = inversekinematics( obj, drone, gimbal, initGuess )
% ========== SOLVING BY NUMERIC METHOD ===================
N = size(obj.num.pos,2);
% 3_O3P = 3_T_f*f_O3P = 3_T_f*(f_O3O + f_OP)
%       = f_T_3^-1 * (-f_d_3 + Pf)
Pf = [obj.sym.pos;1];
P3 = gimbal.num.kin.Tf{3}^-1*(-gimbal.num.kin.Tf{3}(:,4) + Pf);

%syms droneX droneY droneZ objX objY objZ real
theta_1 = zeros(1,N);
theta_2 = zeros(1,N); 
theta_3 = zeros(1,N);
    for i =1:N
        % theta_1 = yaw
        P3 = simplify(expand(subs(P3,gimbal.param.q(2),theta_2(i))));
        % Equations: P3(2:3) = [0;0]
        eqs = P3(2:3);        
        eqs_i = (subs(eqs, drone.sym.pos, drone.num.pos(:,i)));
        eqs_i = (subs(eqs_i, drone.sym.dir, drone.num.dir(:,i) ));
        eqs_i = (subs(eqs_i, obj.sym.pos, obj.num.pos(:,i)));
        eqs_i = vpa(subs(eqs_i));
        if i>1
            sol = vpasolve(eqs_i, [gimbal.param.q(1); gimbal.param.q(3)], [theta_1(i-1); theta_3(i-1)]);
        else
            sol = vpasolve(eqs_i, [gimbal.param.q(1); gimbal.param.q(3)], initGuess);
        end
        theta_1(i) = sol.q_1;
        theta_3(i) = sol.q_3;
    end
theta = [theta_1; theta_2; theta_3];
end

function gimbal_kinematics = forwardkinematics(data,gimbal,drone)
    % Input: 
    %       data - struct of Gimbal parameter
    %       q    - vector of link angles
    %       drone- struct determine position and direction of Drone
    q = gimbal.q; Dq = gimbal.Dq;
    l = data.l; b = data.b; h = data.h;
    n = size(q,1);
    % Rotation matrix
    R = cell(1,n*10+n); %This variable use row cell to store all rotation matrix
    R{10} = rot('z', q(1));
    R{21} = rot('x', q(2));
    R{32} = rot('y', q(3));
    % Direct vectors of origins in stable state: theta = [0 0 0]'
    d0{10} = [-l(1) 0 h(1)]';  % = _O0O1_0
    d0{21} = [l(2) -b(2) 0]'; % = _O1O2_1
    d0{32} = [l(3) b(3) h(3)]'; % = _O2O3_2

    % Direct vectors of origins after rotations
    d = cell(1, n*10+n);
    % Transformation matrix
    T = cell(1, n*10+n);
    for i = 1:n
        d{i*10+i-1} = R{i*10+i-1}*d0{i*10+i-1};
        T{i*10+i-1} = transf(R{i*10+i-1},d{i*10+i-1}); %_i-1_T_i
    end
    i = 1;
    T{i} = simplify( T{i*10+i-1} ); %_T_1 = _0_T_1
    R{i} = T{i}(1:3,1:3);
    d{i} = T{i}(3,1:3);
    for i = 2:n
        T{i} = simplify( T{i-1} * T{i*10+i-1} ); %_T_i = _0_T_i-1 * _i-1_T_i
        R{i} = T{i}(1:3,1:3);
        d{i} = T{i}(3,1:3);
    end
    T = T(1:3);
    R = R(1:3);
    gimbal_kinematics.T = T;
    gimbal_kinematics.R = R;
    %===============
    % Transpose from Gimbal Frame through Quad Frame to Global Fram
    % f_T_0
    % R0 = 
    %R0 = rot('z', drone.dir(3))*rot('y', drone.dir(2))*rot('x', drone.dir(1)) * rot('x',sym('pi'));
    R0 = rot('x', drone.dir(1))*rot('y', drone.dir(2))*rot('z', drone.dir(3)) * rot('x',sym('pi'));
    d0 = drone.pos; %Not calculate the distance from origin of quad to origin of Gimbal
    T0 = transf(R0,d0); %f_T_0
    Tf = cell(1,n+1);
    Rf = cell(1,n+1);
    for i=1:n
        Tf{i} = T0*T{i};
        Rf{i} = Tf{i}(1:3,1:3);
    end
    Tf{4} = T0;
    Rf{4} = R0;
    gimbal_kinematics.Tf = Tf;
    gimbal_kinematics.Rf = Rf;
    
    %============== OMEGA ===========
    o = cell(1,n);
    of = cell(1,n);
    for i = 1:n
        Rd = difftime(R{i},q,Dq);
        Rdf = difftime(Rf{i},q,Dq);
        o{i} = invskew( simplify( Rd * R{i}' ) );
        of{i} = invskew( simplify( Rdf * Rf{i}' ) );
    end
    gimbal_kinematics.o = o;
    gimbal_kinematics.of = of;
    
end

function data = getData(n)
    
    datafile = 'Gimbal_Model_Data_Table.xlsx';
    range1 = 'B3:D5'; %Kinematics Data Table
    range2 = 'B3:K5'; % Dynamics Data Table
    
    [~,~,Params_Kin_Sym] = xlsread(datafile, 1, range1);
    [~,~,Params_Dyn_Sym] = xlsread(datafile, 2, range2);
    [~,~,Params_Kin_Num] = xlsread(datafile, 3, range1);
    [~,~,Params_Dyn_Num] = xlsread(datafile, 4, range2);    
    
    % === SYMBOLIC VALUE ===
    data.sym.l = sym(Params_Kin_Sym(1:n,1)','real');
    data.sym.b = sym(Params_Kin_Sym(1:n,2)','real');
    data.sym.h = sym(Params_Kin_Sym(1:n,3)','real');
    data.sym.g = [0 0 -sym('g','real')]'; %OXYZ
    data.sym.m = sym(Params_Dyn_Sym(1:n,4)','real');
    
    % === NUMERIC VALUE ===
    data.num.l = [Params_Kin_Num{1:n,1}];
    data.num.b = [Params_Kin_Num{1:n,2}];
    data.num.h = [Params_Kin_Num{1:n,3}];
    data.num.g = [0 0 -9.81]'; %OXYZ
    data.num.m = [Params_Dyn_Num{1:n,4}];    

    rC = cell(1,n);
    I = cell(1,n);
    % === SYMBOLIC VALUE ===
    for i=1:n 
        rC{i} = sym(Params_Dyn_Sym(i,1:3)','real');   
        Ii = sym(Params_Dyn_Sym(i,5:10),'real');
        I{i} = [Ii(1) Ii(4) Ii(6);...
                Ii(4) Ii(2) Ii(5);...
                Ii(6) Ii(5) Ii(3)];        
    end
    data.sym.rC = rC;
    data.sym.I = I;
    
    % === NUMERIC VALUE ===
    for i=1:n 
        rC{i} = [Params_Dyn_Num{i,1:3}]';   
        Ii = [Params_Dyn_Num{i,5:10}];
        I{i} = [Ii(1) Ii(4) Ii(6);...
                Ii(4) Ii(2) Ii(5);...
                Ii(6) Ii(5) Ii(3)];        
    end
    data.num.rC = rC;
    data.num.I = I;    
end

function T = transf(R, d)
    T = [R,d;0,0,0,1];
end

function R = rot(axis, phi)
    if axis == 'x'
        u = [1; 0; 0];
    elseif axis == 'y'
        u = [0; 1; 0];
    elseif axis == 'z'
        u = [0; 0; 1];
    else
        error('Invalid axis');
    end
    R = eye(3)*cos(phi)+u*u'*(1-cos(phi))+skew(u)*sin(phi);
end    

function mat_hat = skew(vec)
	if size(vec,1) ~= 3
		error('Vector must be 3x1');
	end
	mat_hat(1,2) = -vec(3);
	mat_hat(1,3) =  vec(2);
	mat_hat(2,3) = -vec(1);

	mat_hat(2,1) =  vec(3);
	mat_hat(3,1) = -vec(2);
	mat_hat(3,2) =  vec(1);
end

function vec = invskew(mat)
    vec = mat(:,1);
    if size(mat,1) ~= 3 && size(mat,2) ~= 3
        error('matrix must be 3x3');
    end
    vec(1) = mat(3,2);
    vec(2) = mat(1,3);
    vec(3) = mat(2,1);
end

function dA_x = diffmat (A, x)

    [m,p] = size(A);
    n = size(x,1);

    dA_x = sym('dA_x',[m n*p]);
    for i = 1:p
        dA_x(:, (i-1)*n+1 :i*n ) = jacobian(A(:,i),x);
    end
end

function dA_t = difftime (A, x, xd)
    % This function is using to find diff(A,t) if x = x(t) = xd
    % x is symbolic variable in the matrix A
    % x is not symbolic function of time
    % so we can't find diff(A,t)
    % This function will substitute x(t) to x variable in A
    % and find diff(A,t)
    % finally, substitute xd to diff(x,t) in A
    n = size(x, 1);
    if size(x,2) ~= 1
        error('Vector must be 3x1')
    end
    syms t real
    xt = sym('xt_',[n,1]);
    for i = 1:n
        xt(i) = sym(['xt_',char(48+i),'(t)']);
        % Turn of the warning of sym('x(t)')
        [~,warningID] = lastwarn;
        warning('off',warningID);
    end
    A = subs(A,x,xt);
    Ad = diff(A,t);
    Ad = subs(Ad,diff(xt,t),xd);
    dA_t = subs(Ad,xt,x);
    
end