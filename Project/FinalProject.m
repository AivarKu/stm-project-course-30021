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

if 1
    % 2D arc scanner
    figure
% %     subplot(2,2,1)
    scatter3(0,0,0,'b*')
    axis(1*[-0.6 0.6 -0.1 1 -1 1])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on

    %Angles
    N1 = 100;
    N2 = 30;

    N1deg = linspace(-45,45,N1);
%     N2deg = linspace(-30,30,N2);
    N2deg = linspace(-45,45,N2);

    points = zeros(N1*N2,3);

     while(1)
        N = ser.BytesAvailable();
        while(N == 0)
            N = ser.BytesAvailable();
        end

        a = fscanf(ser, '%f %d %d');
        r = a(1);
        n1 = a(2)+1;
        n2 = a(3)+1;
        if abs(r) < 1e-4
           continue;
        end

        th2 = 90-N1deg(n1);
        th1 = 90-N2deg(n2);
        x = r*sind(th1)*cosd(th2);
        y = r*sind(th1)*sind(th2);
        z = r*cosd(th1);
%         plot3(x,y,z,'bo')
%         points(k,1) = x;
%         points(k,2) = y;
%         points(k,3) = z;
        scatter3(x,y,z,'bo')

        drawnow()
    end

%     figure
%     h = animatedline;
%     axis([0 4*pi -1 1])
%
%     for x = 1:1000
%         n1 = n1 + 1;
%         if n1 == N1+1
%            n1 = 1;
%            n2 = n2 + 1;
%
%            if (n2 == N2+1)
%                break
%            end
%         end
%
%         r = 0.5;
%         th1 = N1deg(n1);
%         th2 = N2deg(n2);
%         x = r*cos(th1)*sin(th2);
%         y = r*sin(th1)*sin(th2);
%         z = r*cos(th2);
%         addpoints(h,x,y,z)
%         grid on
%         xlabel('x')
%         ylabel('y')
%         zlabel('z')
%
%
%         drawnow limitrate
%     end

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
