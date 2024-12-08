function uv=unitVector(vector)
    uv=vector/sqrt(sum(vector.^2));
end