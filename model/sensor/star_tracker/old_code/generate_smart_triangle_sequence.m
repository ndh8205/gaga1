function triangle_sequence = generate_smart_triangle_sequence(n)
% generate_smart_triangle_sequence - Pyramid 논문의 smart scanning
%
% Input:
%   n - 관측된 별 개수
%
% Output:
%   triangle_sequence - Nx3 [i, j, k] indices

triangle_sequence = [];

for dj = 1:(n-2)
    for dk = 1:(n-dj-1)
        for i = 1:(n-dj-dk)
            j = i + dj;
            k = j + dk;
            triangle_sequence = [triangle_sequence; i, j, k];
        end
    end
end

end