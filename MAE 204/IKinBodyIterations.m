function [thetalist, success] ...
    = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector
%           position error less than ev.
%
% Original function: IKinBody
% Modified by David Lim, 02/25/25
% Additions: 
% - Prints report for each iteration to command window
% - Saves joint angle iterations as rows of matrix, saved as .csv file

thetalist = thetalist0; % initial guess
i = 0;
maxiterations = 20;
thetaarray = zeros(20,length(thetalist)); % intialize joint angle matrix
Tsb = FKinBody(M, Blist, thetalist); % compute end-effector configuration
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T)); % compute error twist
wb = norm(Vb(1: 3)); % compute angular error magnitude
vb = norm(Vb(4: 6)); % compute linear error magnitude
err = wb > eomg || vb > ev; % error flag
printReport(i,thetalist,Tsb,Vb,wb,vb); % calls function to output report
thetaarray(i+1,:) = thetalist'; % save current joint angles
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;  % update joint angles
    i = i + 1;
    % same computations as above:
    Tsb = FKinBody(M, Blist, thetalist);
    Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    wb = norm(Vb(1: 3));
    vb = norm(Vb(4: 6));
    err = wb > eomg || vb > ev;
    printReport(i,thetalist,Tsb,Vb,wb,vb);
    thetaarray(i+1,:) = thetalist';
end
success = ~ err;
writematrix(thetaarray(1:i+1,:),'IKiterations.csv') % save matrix as csv file
end

function printReport(i,theta,Tsb,Vb,wb,vb)
fprintf('Iteration: %d \n',i) % print interation number
disp('Joint vector:') % print joint vector
disp(theta')
disp('End-effector configuration:')
disp(Tsb) % print end-effector configuration
disp('Error twist:')
disp(Vb') % print error twist
fprintf('Angular error magnitude: %.4f \n',wb) % print angular error magnitude
fprintf('Linear error magnitude: %.4f \n \n',vb) % print linear error magnitude
end