function printModelDimensions(vertices)
    minV = min(vertices);
    maxV = max(vertices);
    dims = maxV - minV;
    
    fprintf('  Dimensions (X × Y × Z): %.2f × %.2f × %.2f units\n', ...
            dims(1), dims(2), dims(3));
    fprintf('  Center: [%.2f, %.2f, %.2f]\n', mean([minV; maxV]));
    fprintf('  Vertices: %d\n', size(vertices, 1));
end