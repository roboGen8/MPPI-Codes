function dy = deriv(t,y,Alon,Blon,R,Q,Clon)
S = reshape(y,size(Alon));  % Reshape input y into matrix
eq = -S*Alon - Alon.'*S + S*Blon*(R\(Blon.'))*S - Clon.'*Q*Clon;  % Do the matrix multiply
dy = eq(:);  % Reshape output as a column vector
end