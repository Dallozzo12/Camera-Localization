function R = quat2rotm( q )

% Normalize and transpose the quaternions
q = normalize_rows(q).';

% Reshape the quaternions in the depth dimension
q = reshape(q,[4 1 size(q,2)]);

s = q(1,1,:);
x = q(2,1,:);
y = q(3,1,:);
z = q(4,1,:);

R = [   1-2*(y.^2+z.^2)   2*(x.*y-s.*z)   2*(x.*z+s.*y)
    2*(x.*y+s.*z) 1-2*(x.^2+z.^2)   2*(y.*z-s.*x)
    2*(x.*z-s.*y)   2*(y.*z+s.*x) 1-2*(x.^2+y.^2)   ];

end