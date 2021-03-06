for k = 1:8;

    data = importdata('lineb2.dat');
    [m,n] = size(data);
    Y = zeros(m,1);
    X = [ones(m,1), data(:,k)];

    data = importdata('linew2.dat');
    [m,n] = size(data);
    Y = [ones(m,1); Y];
    X = [[ones(m,1), data(:,k)]; X];
    
    B(1:2,k) = [X\Y];

end

B