function [taskInfo, numtask, isDeploymentDiagram]=coder_instr_task_info()
    isDeploymentDiagram = 0;
                    taskInfo(1).samplePeriod = 0.0025;
            taskInfo(1).sampleOffset = 0.0;

                taskInfo(1).taskPrio = 40;

                taskInfo(1).taskName = 'BaseRate';
            taskInfo(1).entryPoints = {};
            taskInfo(1).nonFcnCallPartitionName = 'D1';

            taskInfo(2).samplePeriod = 0.005;
            taskInfo(2).sampleOffset = 0.0;

                taskInfo(2).taskPrio = 41;

                taskInfo(2).taskName = ['SubRate' '1'];
            taskInfo(2).entryPoints = {};
            taskInfo(2).nonFcnCallPartitionName = 'D2';

            taskInfo(3).samplePeriod = 1.0;
            taskInfo(3).sampleOffset = 0.0;

                taskInfo(3).taskPrio = 42;

                taskInfo(3).taskName = ['SubRate' '2'];
            taskInfo(3).entryPoints = {};
            taskInfo(3).nonFcnCallPartitionName = 'D3';

            taskInfo(4).samplePeriod = 2.0;
            taskInfo(4).sampleOffset = 0.0;

                taskInfo(4).taskPrio = 43;

                taskInfo(4).taskName = ['SubRate' '3'];
            taskInfo(4).entryPoints = {};
            taskInfo(4).nonFcnCallPartitionName = 'D4';

            taskInfo(5).samplePeriod = -1.0;
            taskInfo(5).sampleOffset = -2.0;

                taskInfo(5).taskPrio = 0;

                taskInfo(5).taskName = ['SubRate' '4'];
            taskInfo(5).entryPoints = {};
            taskInfo(5).nonFcnCallPartitionName = '';




    numtask = 5;
    for i = 1:numtask
    if ( 0 == isnumeric(taskInfo(i).samplePeriod) )
    taskInfo(i).samplePeriod = evalin('base', 'str2double(taskInfo(i).samplePeriod)');
    end
    if ( isempty(taskInfo(i).taskName) )
    taskInfo(i).taskName = ['AutoGen' i ];
    end
    end

end 
