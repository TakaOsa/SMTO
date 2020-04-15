import numpy as np
import scipy as sp
import numpy.matlib
import scipy.special
from numpy.linalg import inv, det
from math import log, exp
from sklearn.datasets.samples_generator import make_blobs

def IWVBEMGMMfitting( X, weights, iteNum, clusterSeedNum ):

    #print(X)
    x = np.transpose(X)
    d, n = x.shape

    #iteNum = 300
    m = clusterSeedNum
    e = np.matrix( np.random.rand(n,m) )
    b0 = 1
    W = np.zeros( (m, d, d) )

    for o in range( 0, iteNum ):
        esum = e.sum( axis=1 )
        e = np.divide( e, np.matlib.repmat( e.sum( axis=1 ), 1, m) )
        weitghed_e = np.multiply( np.matlib.repmat( weights, 1, m ), e )
        #g = np.matrix( e.sum( axis=0 ) )
        g = np.matrix( weitghed_e.sum( axis=0 ) )
        a = 1 + g
        b = b0 + g
        nu = 3 + g
        w = a / a.sum( )

        #xe = x * e
        xe = x * weitghed_e
        c = np.divide( xe, np.matlib.repmat( g, d, 1 ) )
        h = np.divide( xe, np.matlib.repmat( b, d, 1 ) )

        for k in range( 0, m ):
            t1 = x - np.matlib.repmat( np.matrix( c[ :, k ]), 1, n )
            t2 = x - np.matlib.repmat( np.matrix( h[ :, k ]), 1, n )
            #K = np.multiply( t1, np.matlib.repmat( np.transpose( np.matrix( e[ :, k ]) ), d, 1 ) )
            K = np.multiply( t1, np.matlib.repmat( np.transpose( np.matrix( weitghed_e[ :, k ]) ), d, 1 ) )
            W[k, :, :] = inv( np.eye(d) +  K   * np.transpose(t1) + c[:, k] * np.transpose( c[:, k] )*b0*g[0,k] / (b0 + g[0,k])  )
            psi_mat = np.matrix( np.zeros( (1, d) ) )
            for j in range(0, d):
                psi_mat[0, j]= sp.special.psi( (nu[0, k] + 1 - j)*0.5 )
            t3 = psi_mat.sum( ) + log( det( W[k, :,:] ) )
            #print(nu)
            A = np.multiply(  t2,  W[k,  :, :] *t2 )
            e[:, k] = np.transpose( np.exp( t3/2.0 +  sp.special.psi(a[0,k]) -  sp.special.psi( a.sum() ) - d/2.0 / b[0, k] - A.sum( axis=0 ) * nu[0, k] / 2.0 ) )
            #W( :, :, k ) = inv( eye(d) + ( t1 .* repmat( e( :, k)' , [d 1] )  ) * t1' + c(:, k) *c (:, k)' * b0 * g(k) / (b0 + g(k))  );
            #t3 = sum ( psi( ( nu(k) + 1 - [1:d]  ) / 2 )  ) + log( det( W( :, :, k ) ) );
            #e( :, k ) = exp( t3 / 2 + psi( a(k) ) - psi( sum(a) ) -  d / 2 / b( k )  - sum( t2 .* ( W( :, :, k ) *t2  ) )* nu( k ) / 2  );

        w0 = w
        h0 = h
        W0 = W

    z = np.argmax(e, axis=1 )

    c = np.matrix( np.unique( np.asarray(z) ) )
    dim, clusterNum = c.shape
    label = np.zeros( (n, 1) )

    for i in range(0, n):
        for j in range(0, clusterNum):
            if z[i, 0] == c[0, j]:
                label[i, 0] = j


    #print( label )
    print("clusterNum")
    print( clusterNum )
    return label, clusterNum

    #print(e)
    #print(z)
