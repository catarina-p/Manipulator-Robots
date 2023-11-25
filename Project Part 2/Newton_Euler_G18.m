% Esta função calcula a formulação Newton-Euler do manipulador para o
% modelo de dinâmica de corpo rigido.
%
% Catarina Pires 90230
% Ricardo Henriques 90349
% Ultima alteração 28 de Maio 2021
%% ------------------------------------------------------------------------
function tau = Newton_Euler_G18(DH, M, dq, ddq, w0_0, dw0_0, ddp0_0)

JointType = arrayfun(@char, DH(:,end), 'uniform', 0);
S = @(x) [  0 -x(3) x(2);
            x(3) 0 -x(1);
            -x(2) x(1) 0];

wi_i{1} = w0_0;
dwi_i{1} = dw0_0;
ddpi_i{1} = ddp0_0;

z0 = [0; 0; 1];

%% Forward Recursion
for i = 1:size(DH,1)
    Aim1_i{i} =  DHTransf_G18(DH(i,:)); % A01, A12, etc
    Rim1_i{i} = Aim1_i{i}(1:3, 1:3); % R01, R12, etc
    rim1_im1i{i} = Aim1_i{i}(1:3, 4); % r0_01, r1_12, etc
    ri_im1i{i} = Rim1_i{i}' * rim1_im1i{i}; % r1_01, r2_12, etc
    rc_im1{i} = ri_im1i{i} + M.rc{i};    %r1_0c1 etc
end
% Rim1_i{size(DH,1)+1} = eye(3);    
for i = 2:size(DH,1)+1
    if JointType{i-1,1} == 'R' || JointType{i-1,1} == 'r'
        %Velocidade Angular
        wi_i{i} = simplify(Rim1_i{i-1}'*(wi_i{i-1} + dq(i-1)*z0));
        
        %Aceleração Angular
        dwi_i{i} = simplify(Rim1_i{i-1}'*(dwi_i{i-1} + ddq(i-1)*z0 + dq(i-1)*S(wi_i{i-1})*z0));
        
        %Aceleração Linear
        ddpi_i{i} =  simplify(Rim1_i{i-1}'*ddpi_i{i-1} + S(dwi_i{i})*ri_im1i{i-1}...
            + S(wi_i{i})*S(wi_i{i})*ri_im1i{i-1}) ;
        
    elseif JointType{i-1,1} == 'P' || JointType{i-1,1} == 'p'
        %Velocidade Angular
        wi_i{i} = simplify(Rim1_i{i-1}' * wi_i{i-1});
        
        %Aceleração Angular
        dwi_i{i} = simplify(Rim1_i{i-1}' * dwi_i{i-1});
        
        %Aceleração Linear
        ddpi_i{i} =  simplify(Rim1_i{i-1}'*(ddpi_i{i-1} + ddq(i-1)*z0) +...
            2*dq(i-1)*S(wi_i{i})*Rim1_i{i-1}'*z0 + S(dwi_i{i})*ri_im1i{i-1} +...
            S(wi_i{i})*S(wi_i{i})*ri_im1i{i-1});
        
    end
    %Aceleração Linear do Centro de Massa
    ddpici{i} = simplify(ddpi_i{i} + S(dwi_i{i})*M.rc{i-1} + S(wi_i{i})*S(wi_i{i})*M.rc{i-1});
    
end

%% Backward Recursion
fi_i{size(DH,1)+1} = [0; 0; 0];
mui_i{size(DH,1)+1} = [0; 0; 0];
Rim1_i{size(DH,1)+1} = eye(3);

for i = size(DH,1):-1:1
    % Força
    fi_i{i} = simplify(M.m{i}*ddpici{i+1} + Rim1_i{i+1}*fi_i{i+1});
    
    % Momento
    mui_i{i} = simplify(M.I{i}*dwi_i{i+1} + S(wi_i{i+1})*M.I{i}*wi_i{i+1} + ...
        Rim1_i{i+1}*mui_i{i+1} + S(rc_im1{i})*fi_i{i} - ...
        S(M.rc{i})*Rim1_i{i+1}*fi_i{i+1});
end

%% Torques de Atuação
for i = 1:size(DH,1)
    if JointType{i,1} == 'R' || JointType{i,1} == 'r'
        tau(i,1) = simplify(mui_i{i}'*Rim1_i{i}'*z0);
        
    elseif JointType{i,1} == 'P' || JointType{i,1} == 'p'
        tau(i,1) = simplify(fi_i{i}'*Rim1_i{i}'*z0);
        
    end
end


end