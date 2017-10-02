function [X, chi_stats]=ICP(X_guess, P, Z, num_iterations, damping, kernel_threshold, H_odom, camera)

X=X_guess;
chi_stats=zeros(1,num_iterations);
num_inliers=zeros(1,num_iterations);
%error = zeros(2,1);
error = zeros(3,1);
total_error = [];

for iteration=1:num_iterations
    H=zeros(6,6);
    b=zeros(6,1);
    chi_stats(iteration)=0;
    %e = zeros(2,num_iterations);
    e = zeros(3,num_iterations);

    for i=1:size(P,2)
        [e(:,i),J] = errorAndJacobian(X, P(:,i), Z(:,i), camera);
        chi=e(:,i)'*e(:,i);

        if (chi>kernel_threshold)
            e(:,i)=e(:,i)*sqrt(kernel_threshold/chi);
            chi=kernel_threshold;

        else
            num_inliers(iteration)=num_inliers(iteration)+1;
        end
        chi_stats(iteration)=chi_stats(iteration)+chi;
        H=H+J'*J; %*H_odom
        b=b+J'*e(:,i); %H_odom*
    end
    %e
    errors_norm=sqrt(sum(e.^2, 1));
    total_error(iteration)=sum(errors_norm)/size(P,2);

%     if iteration>1
%         if total_error(iteration)>=total_error(iteration-1)
%             damping = damping+0.1;
%         else
%             damping = damping-0.1;
%         end
%     end
% 
%     damping

    H=H+eye(6)*damping;
    dx=-H\b;
    size(dx);
    X=v2t(dx)*X;
    size(v2t(dx));

end

end
