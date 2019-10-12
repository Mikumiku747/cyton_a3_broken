classdef RateProfiler < handle
    % Used for profiling the rate at which a loop runs compared to a target
    % frequency
    properties
        startTime       % Used to make a call paired call to toc
        endTime         % Used to record the final elapsed time
        targetFreq      % Goal Frequency
        count           % Number of times the loop was executed
    end
    
    methods
        function start(obj)
            % Records the start of the loop
            obj.startTime = tic;
        end
        
        function toc(obj)
            % Records the end time of the loop
            obj.endTime = toc(obj.startTime);
        end
        
        function [rate, fraction] = calcStats(obj, count, targetFreq)
            % Calculates the loop statistics
            obj.count = count;
            obj.targetFreq = targetFreq;
            rate = count / obj.endTime;
            fraction = rate / targetFreq;
        end
        
        function showStats(obj)
            % Displays the statistics
            rate = obj.count / obj.endTime;
            fraction = rate * 100 / obj.targetFreq;
            fprintf('Run Frequency: %.2fHz (%.1f%%)\n', rate, fraction);
        end
    end
        
end