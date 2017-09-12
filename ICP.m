function [X, chi_stats]=ICP(X_guess, P, Z, num_iterations, damping, kernel_threshold, H_odom)

X=X_guess;
chi_stats=zeros(1,num_iterations);
num_inliers=zeros(1,num_iterations);

for (iteration=1:num_iterations)
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0;
    
    for (i=1:size(P,2))
        [e,J] = errorAndJacobian(X, P(:,i), Z(:,i));
        chi=e'*e;
        
        if (chi>kernel_threshold)
            e=e*sqrt(kernel_threshold/chi);
            chi=kernel_threshold;
            
        else
            num_inliers(iteration)=num_inliers(iteration)+1;
        end
        chi_stats(iteration)=chi_stats(iteration)+chi;
        H=H+J'*J; %H_odom*
        b=b+J'*e; %H_odom*
        
    end
    H=H+eye(6)*damping;
    dx=-H\b;
    X=v2t(dx)*X;
    
end

end
