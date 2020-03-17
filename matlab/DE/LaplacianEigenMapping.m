function [ x_laplacemap ] = LaplacianEigenMapping( x, k, dim )
%LAPLACIANEIGENMAPPING project a given samples onto the Laplacian eigenmap

n = size( x, 1 );

x = x - repmat( mean(x), [ n, 1 ] );
x2 = sum( x.^2, 2 );
d = repmat( x2, 1, n ) + repmat( x2' , n, 1 ) - 2 *x* x';
[ p, i] = sort( d );
W = sparse( d <= ones( n, 1 )*p( k+1, : ) );
W = (W+W'~= 0);

D = diag( sum( W, 2) ); 
L = D - W;
[ z, v ] = eigs( L, D, dim, 'sm' );

x_laplacemap = ( z - repmat(  mean(z), n, 1 ) ) ./ repmat(  ( max(z) -min(z) ), n, 1 );
%x_laplacemap = z;

end

