clear
close
clc

%LFM脉冲串参数
C = 3.0e8;           %光速(m/s)
PW = 3e-6;           %发射信号时宽
B = 20.0e6;          %发射信号带宽
Fs = 20.0e6;         %采样频率
Fc = 15.6e9;         %雷达射频 15.6GHz
numPulses = 256;     %回波脉冲数
PRT = 22e-6;         %雷达发射脉冲重复周期
Lambda = C/Fc;       %雷达工作波长
PRF=1/PRT;           %雷达发射脉冲重复频率

NoisePower = 0;

%RD图参数
dV = PRF/numPulses*Lambda/2;  %速度分辨率
Vmax = dV * numPulses;        %最大测量速度
dR = C/Fs/2;                    %距离分辨率PRT
Rmax = dR * (PRT*Fs);       %最大测量距离

%无人机点云参数
spatial_resolution = 0.02;      %空间分辨率2cm
body_length = 0.3;              %机身长度30cm
body_width = 0.1;               %机身宽度10cm
body_height = 0.02;             %机身高度10cm
arm_length = 0.3;               %连接臂长度30cm
rotor_length = 0.1;             %旋翼长度10cm
rotateTheta = 0.66;             %假设无人机旋翼转速5000RPM,一个PRT转动角度为0.66度


% %无人机点云测试
% for ii = 1:100
%     cla;
%     [UAV_x,UAV_y,UAV_z] = UAV_point(spatial_resolution,body_length,body_width,body_height,arm_length,rotor_length,ii*rotateTheta);
% 
%     scatter3(UAV_x,UAV_y,UAV_z);
%     % 设置x, y, z轴的显示范围
%     xlim([-1 1]);   % 设置x轴的范围为0到1
%     ylim([-1 1]);   % 设置y轴的范围为0到1
%     zlim([-1 1]);   % 设置z轴的范围为0到1
%     hold on;
% 
%     pause(0.1);
% end

%无人机初始位置信息
UAV_Init_position = [1e3,1e3,1e2];

%目标位置信息
target = zeros(10,2,3);
target(1,1,:) = [1e3,1e3,2e2];
target(1,2,:) = [10,10,0];
target(2,1,:) = [9e2,9e2,2e2];
target(2,2,:) = [-10,-10,0];
target(3,1,:) = [1.1e3,1.1e3,1e2];
target(3,2,:) = [10,5,3];

% 统计有效目标的个数
TargetNumber = 0;
for i = 1:size(target,1)
    if any(target(i,:,:),'all')
        TargetNumber = TargetNumber + 1;
    end
end


%目标轨迹生成
[UAV_position,UAV_velocity] = figure_eight_trajectory(UAV_Init_position,PRT);

totalNumPulses = size(UAV_position,2);

frame = fix(totalNumPulses/numPulses);

%生成LFM信号
[t,signal] = generate_lfm_waveform(PW, B, 0, Fs, 1, PRT);

echo = zeros(100,numPulses,PRT*Fs);
position = zeros(100,20,2);
mtd = zeros(100,numPulses,Fs*PRT);


for timeIndex = 1:100
    for ii = 1:numPulses
        [UAV_x,UAV_y,UAV_z] = UAV_point(spatial_resolution,body_length,body_width,body_height,arm_length,rotor_length,(ii+(timeIndex-1)*numPulses)*rotateTheta);
        x = UAV_x + UAV_position(1,ii+(timeIndex-1)*numPulses);
        y = UAV_y + UAV_position(2,ii+(timeIndex-1)*numPulses);
        z = UAV_z + UAV_position(3,ii+(timeIndex-1)*numPulses);
        for jj = 1:size(x,2)
            UAV_R = norm([x(jj),y(jj),z(jj)]);
            tau = 2*UAV_R/C;
            delay = fix(tau*Fs);
            signal_temp = signal*exp(1i*(2*pi*Fc*tau));    
            echo_dot = [zeros(1,delay),signal_temp(1,1:size(signal_temp,2)-delay)];
            echo(timeIndex,ii,:) = echo(timeIndex,ii,:) + reshape(echo_dot,1,1,[]);
        end
        echo(timeIndex,ii,:) = echo(timeIndex,ii,:) ./ 50;


        for jj = 1:TargetNumber
            target(jj,1,:) = target(jj,1,:) + target(jj,2,:) * PRT;
            target_R = norm(reshape(target(jj,1,:),[1,3]));
            tau = 2*target_R/C;
            delay = fix(tau*Fs);
            signal_temp = signal*exp(1i*(2*pi*Fc*tau));    
            echo_dot = [zeros(1,delay),signal_temp(1,1:size(signal_temp,2)-delay)];
            echo(timeIndex,ii,:) = echo(timeIndex,ii,:) + reshape(echo_dot,1,1,[]);
        end
        SystemNoise=normrnd(0,10^(NoisePower/10),1,PRT*Fs)+1j*normrnd(0,10^(NoisePower/10),1,PRT*Fs);%均值为0，标准差为10^(NoisePower/10)的噪声
        echo(timeIndex,ii,:) = echo(timeIndex,ii,:) + reshape(SystemNoise,1,1,[]);
    end



    %频域脉冲压缩
    fft_length = 512; % 保持 512 点的 FFT

    % 构造汉明窗
    hamming_window = hamming(fft_length).'; % 确保是行向量


    % 匹配滤波器构造并应用汉明窗
    signal_padded = [signal, zeros(1, fft_length - length(signal))];
    coeff = conj(fliplr(signal_padded));
    coeff_fft = fft(coeff.*hamming_window, fft_length);

    % 回波信号补零并压缩
    echo_compressed = zeros(numPulses,fft_length);
    for ii = 1:numPulses
        % 为回波信号补零以匹配 512 点 FFT
        echo_temp = [reshape(echo(timeIndex,ii,:),[1,440]),zeros(1,fft_length-length(echo(timeIndex,ii,:)))];
        echo_fft = fft(echo_temp.* hamming_window, fft_length);
        echo_compressed(ii,:) = ifft(echo_fft.*coeff_fft);
    end


    % 构造速度维窗函数（针对脉冲数）
    doppler_window = hamming(numPulses).'; % 确保为行向量

    for i=1:440
        buff(1:numPulses)=echo_compressed(1:numPulses,i);
        buff_fft=fft(buff.*doppler_window);
        buff_fft = fftshift(buff_fft);
        mtd(timeIndex,1:numPulses,i)=buff_fft(1:numPulses);
    end

    CFAR_position = CA_CFAR(reshape(mtd(timeIndex,:,:),[256,440]),2,2);
    position(timeIndex,1:size(CFAR_position,1),:) = CFAR_position;


    if timeIndex == 1
        % 在第一次循环时创建图像
        figure;
        h = mesh(abs(reshape(mtd(timeIndex,:,:),[256,440])));
    else
        % 更新图像的数据
        h.ZData = abs(reshape(mtd(timeIndex,:,:),[256,440]));  % 更新 mesh 图像的 Z 数据
    end

    % 强制刷新图像
    drawnow;  % 刷新图形窗口，确保图像更新

    % 停顿0.5秒
    pause(0.1);  % 暂停0.5秒

end