clc
close all
clear
format compact
%% Created on 27.10.2021
COM_PORT = 'COM4';

oldSerial = instrfind('Port', COM_PORT); 
if (~isempty(oldSerial))  % if the set of such objects is not(~) empty 
    disp('WARNING:  COM in use.  Closing.') 
    delete(oldSerial) 
end 

ser = serial(COM_PORT, 'baudrate', 9600, 'terminator', 'CR');
fopen(ser);

if 0
    % 2D arc scanner
    figure
    subplot(2,2,1)
    hold on
    
    %addpoints
    
    while(1)
        
    end
    
else
    disp('1D lidar stared')
    % 1D radar
    polarscatter(0,0)
    hold on
    rax = [0 0.9];

    theta = 0:1e-1:2*pi;
    thetaLen = length(theta);
    thetaCnt = 1;
    rho = zeros(1,thetaLen);

    while(1)
        N = ser.BytesAvailable();
        while(N == 0)
            N = ser.BytesAvailable();
        end

        a = fscanf(ser, '%f');
        if abs(a) < 1e-4
           continue; 
        end

        rho(thetaCnt) = a;

        clf
        polarplot([0 theta(thetaCnt) 0 theta],[0 3 0 rho],'b')
        rlim(rax)

        thetaCnt = thetaCnt + 1;
        if (thetaCnt == thetaLen)
            thetaCnt = 1;
        end
        drawnow()
    end
end

delete(ser)
clear ser 
