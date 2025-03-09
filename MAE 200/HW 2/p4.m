% Note: A & A_hat are not diagonalizable

A = [0 0 0; 1 0 -1; 0 1 -2];
[U1,J1] = jordan(A);

A_hat = [0 0 0; 1 -1 0; 0 1 -1];
[U2,J2] = jordan(A_hat);

T = U2*U1^-1