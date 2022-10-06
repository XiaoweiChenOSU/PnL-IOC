%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%   R�cup�rer les donn�es utilisateur
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [UserInput, GAParameters] = GetUserInput()

disp('Algorithme g�n�tique multi-objectif');
disp('------------------------------------------------------------');

%-----------------------------------------
%   Choix probl�me
%-----------------------------------------
disp('I - Probl�me � traiter');
UserInput.Probleme = input('     1 = SCH, 2 = FON, 3 = POL, 4 = KUR, 5 = ZDT1, 6 = ZDT2, 7 = ZDT3, 8 = ZDT4, 9 = ZDT6\n     Votre choix : ');
%Test validit�
if (UserInput.Probleme ~= 1 && UserInput.Probleme ~= 2 && UserInput.Probleme ~= 3 && UserInput.Probleme ~= 4 && UserInput.Probleme ~= 5 && UserInput.Probleme ~= 6 && UserInput.Probleme ~= 7 && UserInput.Probleme ~= 8 && UserInput.Probleme ~= 9)
    disp('     Choix non valide -> Probl�me 1 = SCH choisi par d�faut');
    UserInput.Probleme = 1;
end

%-----------------------------------------
%   Choix probl�me
%-----------------------------------------
disp('II - Algorithme � utiliser');
UserInput.Algorithme = input('     1 = SPEA2, 2 = NGSA-II\n     Votre choix : ');
%Test validit�
if (UserInput.Algorithme ~= 1 && UserInput.Algorithme ~= 2)
    disp('     Choix non valide -> Algorithme 1 = SPEA2 choisi par d�faut');
    UserInput.Algorithme = 1;
end

%-----------------------------------------
%   Param�tres g�n�raux
%-----------------------------------------
disp('III - Param�tres g�n�raux du probl�me');
GAParameters.PopSize = input('     Taille de population (N) : ');
GAParameters.ArchiveSize = input('     Taille archive (NA) : ');
GAParameters.Gmax = input('     Nombre de g�n�rations max (Gmax) : ');
GAParameters.Pc = input('     Probabilit� de croisement Pc (entre 0 et 1) : ');
GAParameters.Pm = input('     Probabilit� de mutation Pm (entre 0 et 1) : ');

%-----------------------------------------
%   Autres param�tres appliqu�s
%-----------------------------------------
disp('IV - Autres param�tres appliqu�s par d�faut');
disp('     S�lection : Par tournoi binaire');
disp('     Croisement : Simulated Binary');
disp('     Mutation : Polynomial');



disp('V - Lancement');
disp('------------------------------------------------------------');