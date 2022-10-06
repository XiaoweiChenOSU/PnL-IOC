%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Selection by tournament
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function MP = SelectionTournoi67(individuVide, P)

%-----------------------------------------
%   Init variables
%-----------------------------------------
k = 2; %Number of individuals competing in the tournament (binary)




F = [P.Rank];

N = size(P,1);
MP = repmat(individuVide, N, 1);


%-----------------------------------------
%  Path of the number of individuals to be selected (N in the general case)
%-----------------------------------------
for i = 1:N

    kj = [];
    
    %Generation of k random permutations
    for j = 1:k
        kj = [kj; randperm(N)];
    end
    
    if F(kj(1,i)) > F(kj(2,i))
        MP(i) = P(kj(1,i));
    else
        MP(i) = P(kj(2,i));
    end
%     if P(kj(1,i)).F > P(kj(2,i)).F
%         MP(i) = P(kj(1,i));
%     else
%         MP(i) = P(kj(2,i));
%     end

end