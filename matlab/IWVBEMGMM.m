function [ z, W_opt, w_opt, h_opt, nu_opt, a_opt ] = IWVBEMGMM( x, seedsNum, z_ini, weights, iteNum )
%VBEMGMM performs GMM fitting based on the variational Bayes EM algorithm.
%This function returns the latent variale for each sample

    [ d, n ] = size( x ); 
    m = seedsNum; 
    e = rand( n, m ); 
    %e = z_ini;

    W = zeros( d, d, m ); 
    b0 = 1;
    %D = repmat(weights, d, 1);

    for o=1:iteNum
        e = e./repmat( sum( e, 2 ), [1 m] );
        %g = sum(e);
        g = sum( repmat(weights', 1, m ) .* e);
        a = 1 + g;
        b = b0 + g;
        nu = 10 + g; %3
        w = a / sum (a);

        %xe = x*e;
        xe = x* ( repmat(weights', 1, m ) .* e );
        c = xe ./ repmat( g, [d 1] );
        h = xe ./ repmat( b, [d 1] );

        for k = 1:m
            t1 = x - repmat( c( :, k ), [1 n] );
            t2 = x - repmat( h( :, k ), [1 n] );
            %W( :, :, k ) = inv( eye(d) + ( t1 .* repmat( e( :, k)' , [d 1] )  ) * t1' + c(:, k) *c (:, k)' * b0 * g(k) / (b0 + g(k))  );
            W( :, :, k ) = inv( eye(d) + ( t1 .* repmat( weights .* e( :, k)' , [d 1] )  ) * t1' + c(:, k) *c (:, k)' * b0 * g(k) / (b0 + g(k))  );
            t3 = sum ( psi( ( nu(k) + 1 - [1:d]  ) / 2 )  ) + log( det( W( :, :, k ) ) );
            e( :, k ) = exp( t3 / 2 + psi( a(k) ) - psi( sum(a) ) -  d / 2 / b( k )  - sum( t2 .* ( W( :, :, k ) *t2  ) )* nu( k ) / 2  );
        end
        if o > 1 && norm( w - w0 ) + norm( h - h0 ) + norm( W( : ) - W0( : )  ) < 0.0001
            break;
        end
        w0 = w; 
        h0 = h;
        W0 = W;
    end

    e = e./repmat( sum( e, 2 ), [1 m] );

    [~, z] = max( e, [], 2 );

    labelVal =  unique(z);
    clusterNum = size(labelVal, 1);
    label = zeros(n, 1);
    
    for i = 1:n
        for j = 1:clusterNum
            if z(i, 1) == labelVal( j )
                label( i, 1 ) = j;
            end
        end
    end
    
    %z = z';
    z = label';
    W_opt = W;
    w_opt = w;
    h_opt = h;
    nu_opt = nu;
    a_opt = a;
end

