function row_norm = normalize_rows(matrix)
% Returns a matrix the same size as the input MATRIX, with each row normalized to a unit vector
% with length of 1: divide each vector element by the euclidean norm of the whole vector.

row_norm = bsxfun(@times, matrix, 1./sqrt(sum(matrix.^2,2)));

end