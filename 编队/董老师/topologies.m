function G = topologies(E, checkLemma1)

if nargin < 2
    % turn off Lemma 1 check by default
    checkLemma1 = true;
end

N = 8;

assert(max(E,[],'all') <= N, 'edge set E contains node with index > N')

%% graph matrix generation
W = zeros(N, N);
D = zeros(N, 1);
nEdge = size(E,1);


for idxEdge = 1:nEdge
    ee = E(idxEdge, :);
    W(ee(2), ee(1)) = 1;
end

for idxNode = 1:N
    D(idxNode) = sum(W(idxNode,:)~=0);
end

D = diag(D);

L = D - W;

Neigh = cell(1,N);

for idxNode = 1:N
    Neigh{idxNode} = find(W(idxNode,:), N-1);
end

if checkLemma1
    %{
    Lemma 1:
    If G contains a spanning tree, then 0 is a simple eigenvalue of L with
    associated right eigenvector 1, and all other N-1 eigenvalues have positive
    real parts.

    Consensus seeking in multiagent systems under dynamically changing
    interaction topologies
    https://ieeexplore.ieee.org/document/1431045
    %}
    e = eig(L);
    e_re = real(e);
    assert(sum(abs(e_re) < 4*eps) == 1 && sum(e_re >  4*eps) == N-1,...
        'No spanning tree!')
end

G.E = E;
G.W = W;
G.D = D;
G.L = L;
G.N = Neigh;

end