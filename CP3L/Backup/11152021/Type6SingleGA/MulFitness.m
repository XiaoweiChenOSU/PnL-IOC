% Fitness (objective function)
function f = Fitness(x)


    % De Jong's Function 3
    f = abs(x{1}) + abs(x{2}) + abs(x{3}) + abs(x{4}) + abs(x{5});

    % De Jong?s Function 5
%     a = [-32 -16 0 16 32 -32 -16 0 16 32 -32 -16 0 16 32 -32 -16 0 16 32 -32 -16 0 16 32;...
%          -32 -32 -32 -32 -32 -16 -16 -16 -16 -16 0 0 0 0 0 16 16 16 16 16 32 32 32 32 32];
%     f = 0.002;
%     for j = 1:25
%         t = j + (x1-a(1,j)).^6 + (x2-a(2,j)).^6;
%         f = f + 1/t;
%     end

    % Rastrigin's Function
%     n = 2;
%     f = 10*n
%     for i = 1:n
%         f = f + 
%     end    
end