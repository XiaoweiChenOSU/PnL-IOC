%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Display results
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ResultatsDisplay71(Front, G)

%Display of generation number and front size
disp(['G = ' num2str(G) ' // Front Size = ' num2str(size(Front,1))]);

%Forehead plot
figure(1);
ValObjPF = [Front.ValObjective];
% disp('Min distances seperately are:  ');
% disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
plot(ValObjPF(1,:),ValObjPF(2,:),'co');
xlabel('f1');
ylabel('f2');
% elseif ProblemId == 9
%     %Forehead plot
%     figure(1);
%     ValObjPF = [Front.ValObjective];
%     % disp('Min distances seperately are:  ');
%     % disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
%     plot(ValObjPF(1,:),ValObjPF(3,:),'co');
%     xlabel('f1');
%     ylabel('f3');
% elseif ProblemId == 10
%     %Forehead plot
%     figure(1);
%     ValObjPF = [Front.ValObjective];
%     % disp('Min distances seperately are:  ');
%     % disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
%     plot(ValObjPF(1,:),ValObjPF(4,:),'co');
%     xlabel('f1');
%     ylabel('f4');
% elseif ProblemId == 11
%     %Forehead plot
%     figure(1);
%     ValObjPF = [Front.ValObjective];
%     % disp('Min distances seperately are:  ');
%     % disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
%     plot(ValObjPF(2,:),ValObjPF(3,:),'co');
%     xlabel('f2');
%     ylabel('f3');
% elseif ProblemId == 12
%     %Forehead plot
%     figure(1);
%     ValObjPF = [Front.ValObjective];
%     % disp('Min distances seperately are:  ');
%     % disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
%     plot(ValObjPF(2,:),ValObjPF(4,:),'co');
%     xlabel('f2');
%     ylabel('f4');
% elseif ProblemId == 13
%     %Forehead plot
%     figure(1);
%     ValObjPF = [Front.ValObjective];
%     % disp('Min distances seperately are:  ');
%     % disp(['d1 = ' num2str(ValObjPF(1,:)) ' // d2 = ' num2str(ValObjPF(2,:))  ' // d3 = ' num2str(ValObjPF(3,:))  ' // d4 = ' num2str(ValObjPF(4,:))]);
%     plot(ValObjPF(3,:),ValObjPF(4,:),'co');
%     xlabel('f3');
%     ylabel('f4');
% end


title('Camera Pose Estimation');    
grid on;
pause(0.01);



% if ProblemId == 7
% 
%     figure(2);
%     plot(ValObjPF(3,:),ValObjPF(4,:),'ro');
%     xlabel('f3');
%     ylabel('f4');
% 
%     switch ProblemId
%         %ZDT1
%         case 1
%             title('ZDT1 Result');
%         %ZDT2
%         case 2
%             title('ZDT2 Result');
%         %ZDT3
%         case 3 
%             title('ZDT3 Result');
%         %ZDT4
%         case 4
%             title('ZDT4 Result');
%         %ZDT5
%         case 5
%             title('ZDT5 Result');
%         %ZDT6
%         case 6
%             title('ZDT6 Result');
%         case 7
%             title('Camera Pose Estimation'); 
%         case 8
%             title('Camera Pose Estimation');    
%         case 9
%             title('Camera Pose Estimation');    
%         case 10
%             title('Camera Pose Estimation');    
%         case 11
%             title('Camera Pose Estimation');    
%     end
%     grid on;
%     pause(0.01);
% end