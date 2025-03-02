 function [t,signal] = generate_lfm_waveform(PW, B, Fc, Fs, numPulses, PRT)
    %计算总时间长度
    totalTime = numPulses*PRT;
    %创建总时间轴
    t = -totalTime/2:1/Fs:totalTime/2-1/Fs;
    %初始化脉冲串数组
    signal = zeros(size(t));

    for k= 0:numPulses-1
        %计算当前脉冲的起始和结束时间  
        pulseStart = k * PRT - totalTime/2;
        pulseEnd = pulseStart + PW;

        %在时间轴上找到对应当前脉冲的索引  
        pulseIndices = (t >= pulseStart) & (t < pulseEnd);  

        %计算当前脉冲的时间轴  
        t_pulse = t(pulseIndices) - pulseStart;  
          
        % 如果当前时间在脉冲范围内，则生成LFM信号  
        if any(pulseIndices)  
            % 计算调频斜率，默认正调频斜率
            kT = B / PW;  
              
            % 生成线性调频信号  
            phase = pi * kT * (t_pulse ).^2;  
            s_pulse = exp(1i * (2 * pi * (Fc - B/2) * t_pulse + phase));  

            % 将脉冲添加到脉冲串数组中  
            signal(pulseIndices) = s_pulse;  
        end  
    end

    % % 可选：绘制时域波形  
    % figure;  
    % subplot(2,1,1);  
    % plot(t, real(signal));  
    % title('时域波形');  
    % xlabel('时间 (s)');  
    % ylabel('幅度');  
    % grid on;  
    % 
    % % 可选：绘制频域波形  
    % f = linspace(-Fs/2, Fs/2, length(t));  
    % signal_FFT = fftshift(fft(signal));  
    % subplot(2,1,2);  
    % plot(f, abs(signal_FFT));  
    % title('频域波形');  
    % xlabel('频率 (Hz)');  
    % ylabel('幅度');  
    % grid on;  
end