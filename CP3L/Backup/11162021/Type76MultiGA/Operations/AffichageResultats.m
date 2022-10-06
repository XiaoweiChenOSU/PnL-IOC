%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Affichage r�sultats
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function AffichageResultats(Front, G)

%Affichage de Nb g�n�ration et de la taille du front
disp(['G = ' num2str(G) ' // Taille Front = ' num2str(size(Front,1))]);

%Plot du front
figure(1);
ValObjPF = [Front.ValObjective];
plot(ValObjPF(1,:),ValObjPF(2,:),'x');
xlabel('1^{er} Objectif');
ylabel('2^{�me} Objectif');
grid on;
pause(0.01);