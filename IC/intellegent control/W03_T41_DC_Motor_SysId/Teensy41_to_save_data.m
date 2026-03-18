%% 41277 Control Design
%  Teensy 4.1 - Save test data
%
%  This script will run after sttoping the Simulink Model. 

Nini = 1;
time = logsout{1}.Values.Time(Nini:end);
Vm = logsout{1}.Values.Data(Nini:end);
theta = logsout{2}.Values.Data(Nini:end);
w = logsout{3}.Values.Data(Nini:end);

%% save data.
file_name = "Teensy41_test_data";
count = 1;

check_file = true;
while(check_file)
    if count<10
        file_full = file_name + "_0" + num2str(count) + ".mat";    
    else
        file_full = file_name + "_" + num2str(count) + ".mat";   
    end
    if  (exist(file_full,'file')>0)
        count = count + 1;     
    else 
        check_file = false;
    end
end
save(file_full,'time','logsout');



