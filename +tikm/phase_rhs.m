function v = phase_rhs(theta,c,adj,stopped)
%PHASE_RHS Paper Eq. (1), no normalization by the number of neighbors.
v=c.omega*ones(c.N,1);
if c.feedback
    delta=theta.'-theta; % row i, column j = theta_j - theta_i
    v=v-sum(adj.*sin(delta),2);
end
v(stopped)=0;
end
