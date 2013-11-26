function r = ranks(A,tol)
%RANK   Matrix rank.
%   RANK(A) provides an estimate of the number of linearly
%   independent rows or columns of a matrix A.
%   RANK(A,tol) is the number of singular values of A
%   that are larger than tol.
%   RANK(A) uses the default tol = max(size(A)) * eps(norm(A)).
%
%   Class support for input A:
%      float: double, single

%   Copyright 1984-2007 The MathWorks, Inc.
%   $Revision: 5.11.4.5 $  $Date: 2007/08/03 21:26:23 $

s = svds(A);
if nargin==1
   tol = max(size(A)) * eps(max(s));
end
r = sum(s > tol);
