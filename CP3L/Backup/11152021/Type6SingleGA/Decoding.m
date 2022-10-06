function [x1,x2,x3,x4] = Decoding(Genes,lens,range)
    [row, ~] = size(Genes);
    d = zeros(row, 1);
    for i=1:lens(1)
        d = d + Genes(:,i)*2^(lens(1)-i);
    end
    x1 = d * (range{1}(2) - range{1}(1)) / 2^lens(1) + range{1}(1);
    
    d = zeros(row, 1);
    for i=lens(1)+1:lens(1)+lens(2)
        d = d + Genes(:,i)*2^(lens(2)-(i-lens(1)));
    end
    x2 = d * (range{2}(2) - range{2}(1)) / 2^lens(2) + range{2}(1);
    
    d = zeros(row, 1);
    for i=lens(1)+lens(2)+1:lens(1)+lens(2)+lens(3)
        d = d + Genes(:,i)*2^(lens(3)-(i-lens(1)-lens(2)));
    end
    x3 = d * (range{3}(2) - range{3}(1)) / 2^lens(3) + range{3}(1);
     
    d = zeros(row, 1);
    for i=lens(1)+lens(2)+lens(3)+1:lens(1)+lens(2)+lens(3)+lens(4)
        d = d + Genes(:,i)*2^(lens(4)-(i-lens(1)-lens(2)-lens(3)));
    end
    x4 = d * (range{4}(2) - range{4}(1)) / 2^lens(4) + range{4}(1);
      
end