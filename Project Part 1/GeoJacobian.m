function [J] = GeoJacobian (DH,JointType)
% Esta fun��o c�lcula matriz Jacobiana Geom�trica do rob�.
%
% Inputs: - DH: tabela Denavit-Hartenberg do rob� (simb�lico);
%         - JointType: string que indica o tipo de junta, para cada junta.
%         Deve ter o numero de caracteres igual ao n�mero de juntas, e
%         sempre que uma junta � prism�tica ou de revolu��o o respectivo
%         caracter deve ser 'P' ou 'R'. N�o � case sensitive.
%
% Outputs: - J: Matriz Jacobiana Geom�trica do rob� em fun��o dos �ngulos 
%         nas juntas (simb�lico);
%
% Catarina Pires 90230
% Ricardo Henriques 90349
% �ltima altera��o a 23 de Abril 2021
% ------------------------------------------------------------------------
all_z{1} = [0;0;1]; %z_0
p_i_1{1} = [0;0;0;1]; %_p_0

a = 2;

% C�lculo de p_{i-1} e z_{i-1}
while a <= size(DH, 1)+1
    A0am1 = DKin_G18(DH(1:a-1,:));
    p_i_1{a} = A0am1*p_i_1{1};
    all_z{a} = A0am1(1:3,1:3)*all_z{1};
    a= a+1;
end

% Calculo de p_e
pe = A0am1*p_i_1{1};

% Calculo da Jacobiana coluna a coluna de acordo com o tipo de junta
for k = 1:size(DH, 1)
    if JointType(k) == 'R' || JointType(k) == 'r'
        J (:,k) = [cross(all_z{k},(pe(1:3,1)-p_i_1{k}(1:3,1))); all_z{k}];        
    elseif JointType(k) == 'P' || JointType(k) == 'p'
        J (:,k) = [all_z{k}; 0];        
    end
end

end
