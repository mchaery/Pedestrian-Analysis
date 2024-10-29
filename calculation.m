function[] = calculation(Case_name, Video_view)

Raw_file_name=strcat(Case_name,'\raw_data.csv');
outcome_file_name=strcat(Case_name,'\output\',Case_name,'_result');
mkdir(strcat(Case_name,'\output'));

%% 기본 분석 변수 설정


reference_data=importdata('reference_CES48.csv');
HSTO_landmark='A'; % A (ankle), H (Heel), F (Foot)
if HSTO_landmark =='A'
    MPD=10;
    MPD_gain=0.7;
elseif HSTO_landmark=='H'
    MPD=10;
    MPD_gain=0.7;
elseif HSTO_landmark=='F'
    MPD=10;
    MPD_gain=0.7;
end


wp_kph=4.0;
wp_gain=1;

kalman_onoff=1;
kalmanplot_onoff=0;

gaitfeatures_onoff=1;
HSTOcheck_onoff=1;


LPF_onoff=1;
LPF_fq=1;
fft_plot_onoff=1;



%% 1 Read MP Pose outcome
Header=importdata(Raw_file_name).colheaders;
raw_data=importdata(Raw_file_name).data;
[m,n]=size(raw_data);

dynamicdata=raw_data;
%% 1.1 Data 축 변환
if Video_view=='B'
    dynamicdata(:,2:4:132)=raw_data(:,4:4:132);
    dynamicdata(:,3:4:132)=-raw_data(:,2:4:132);
    dynamicdata(:,4:4:132)=-raw_data(:,3:4:132);
    ans='Back'
elseif Video_view=='R'
    dynamicdata(:,2:4:132)=raw_data(:,2:4:132);
    dynamicdata(:,3:4:132)=raw_data(:,4:4:132);
    dynamicdata(:,4:4:132)=-raw_data(:,3:4:132);
    ans='Right'
else

end


%% 1.2 Extend data (for Kalman Filter)

time_base = dynamicdata(:,1);
mat_base = dynamicdata(:,2:end);
data_base = [time_base mat_base];


sec_gaitend=time_base(end)-10;
sec_gaitstart=sec_gaitend-20;

time_ext = [-5.00:0.05:-0.05]';
te = length(time_ext);
for i = 1:te
    mat_new(i,:) = dynamicdata(1,2:end);
end
data_ext = [time_ext mat_new];

dynamicdata = [data_ext; data_base];
time = dynamicdata(:,1);
[md nd] = size(dynamicdata);

%% 1.3 Data 이름 붙이기

LTCT_mass = 10.34+9.54; % kg 
UT2_mass = 5.67;
UT1_mass = 8.51;
neckseg_mass = 1.30; 
headseg_mass = 9.40;

UA_L_mass = 2.78;
LA_L_mass = 2.01;
UA_R_mass = 2.78;
LA_R_mass = 2.01;

handseg_L_mass = 0; 
handseg_R_mass = 0;

UL_L_mass = 9.58;
LL_L_mass = 3.53;
UL_R_mass = 9.58;
LL_R_mass = 3.53;

upperbody_mass = LTCT_mass+UT2_mass+UT1_mass+neckseg_mass+headseg_mass+UA_L_mass+LA_L_mass+UA_R_mass+LA_R_mass+handseg_L_mass+handseg_R_mass;
body_mass = upperbody_mass+UL_L_mass+LL_L_mass+UL_R_mass+LL_R_mass;


hip_L_raw = dynamicdata(:,94:96);
hip_R_raw = dynamicdata(:,98:100);

pelvis_raw = (hip_L_raw+hip_R_raw)/2;


shoulder_L_raw = dynamicdata(:,46:48);
elbow_L_raw = dynamicdata(:,54:56);
wrist_L_raw = dynamicdata(:,62:64);
% hand_L_raw = dynamicdata(:,67:69);


shoulder_R_raw = dynamicdata(:,50:52);
elbow_R_raw = dynamicdata(:,58:60);
wrist_R_raw = dynamicdata(:,66:68);
% hand_L_raw = dynamicdata(:,67:69);


nose_raw = dynamicdata(:,2:4);
eye_L_raw = dynamicdata(:,10:12);
eye_R_raw = dynamicdata(:,22:24);
ear_L_raw = dynamicdata(:,30:32);
ear_R_raw = dynamicdata(:,34:36);
mouth_L_raw = dynamicdata(:,38:40);
mouth_R_raw = dynamicdata(:,42:44);


knee_L_raw = dynamicdata(:,102:104);
ankle_L_raw = dynamicdata(:,110:112);
foot_L_raw = dynamicdata(:,126:128);
heel_L_raw = dynamicdata(:,118:120);

knee_R_raw = dynamicdata(:,106:108);
ankle_R_raw = dynamicdata(:,114:116);
foot_R_raw = dynamicdata(:,130:132);
heel_R_raw = dynamicdata(:,122:124);

%% 2 Kalman Filter

%% 2.1 if (Kalman filter 적용)
if kalman_onoff == 1

    count = 1;

    for i = 1:md

        [pelvis(i,1),pelvis(i,2),pelvis(i,3),count] = TrackKalman3d(pelvis_raw(i,1),pelvis_raw(i,2),pelvis_raw(i,3),1,count);
        [hip_L(i,1),hip_L(i,2),hip_L(i,3),count] = TrackKalman3d(hip_L_raw(i,1),hip_L_raw(i,2),hip_L_raw(i,3),6,count);
        [hip_R(i,1),hip_R(i,2),hip_R(i,3),count] = TrackKalman3d(hip_R_raw(i,1),hip_R_raw(i,2),hip_R_raw(i,3),7,count);


        [shoulder_L(i,1),shoulder_L(i,2),shoulder_L(i,3),count] = TrackKalman3d(shoulder_L_raw(i,1),shoulder_L_raw(i,2),shoulder_L_raw(i,3),9,count);
        [elbow_L(i,1),elbow_L(i,2),elbow_L(i,3),count] = TrackKalman3d(elbow_L_raw(i,1),elbow_L_raw(i,2),elbow_L_raw(i,3),10,count);
        [wrist_L(i,1),wrist_L(i,2),wrist_L(i,3),count] = TrackKalman3d(wrist_L_raw(i,1),wrist_L_raw(i,2),wrist_L_raw(i,3),11,count);
        %         [hand_L(i,1),hand_L(i,2),hand_L(i,3),count] = TrackKalman3d(hand_L_raw(i,1),hand_L_raw(i,2),hand_L_raw(i,3),12,count);


        [shoulder_R(i,1),shoulder_R(i,2),shoulder_R(i,3),count] = TrackKalman3d(shoulder_R_raw(i,1),shoulder_R_raw(i,2),shoulder_R_raw(i,3),14,count);
        [elbow_R(i,1),elbow_R(i,2),elbow_R(i,3),count] = TrackKalman3d(elbow_R_raw(i,1),elbow_R_raw(i,2),elbow_R_raw(i,3),15,count);
        [wrist_R(i,1),wrist_R(i,2),wrist_R(i,3),count] = TrackKalman3d(wrist_R_raw(i,1),wrist_R_raw(i,2),wrist_R_raw(i,3),16,count);
        %         [hand_R(i,1),hand_R(i,2),hand_R(i,3),count] = TrackKalman3d(hand_R_raw(i,1),hand_R_raw(i,2),hand_R_raw(i,3),17,count);

        [nose(i,1),nose(i,2),nose(i,3),count] = TrackKalman3d(nose_raw(i,1),nose_raw(i,2),nose_raw(i,3),18,count);
        [eye_L(i,1),eye_L(i,2),eye_L(i,3),count] = TrackKalman3d(eye_L_raw(i,1),eye_L_raw(i,2),eye_L_raw(i,3),19,count);
        [eye_R(i,1),eye_R(i,2),eye_R(i,3),count] = TrackKalman3d(eye_R_raw(i,1),eye_R_raw(i,2),eye_R_raw(i,3),20,count);
        [ear_L(i,1),ear_L(i,2),ear_L(i,3),count] = TrackKalman3d(ear_L_raw(i,1),ear_L_raw(i,2),ear_L_raw(i,3),21,count);
        [ear_R(i,1),ear_R(i,2),ear_R(i,3),count] = TrackKalman3d(ear_R_raw(i,1),ear_R_raw(i,2),ear_R_raw(i,3),22,count);


        [knee_L(i,1),knee_L(i,2),knee_L(i,3),count] = TrackKalman3d(knee_L_raw(i,1),knee_L_raw(i,2),knee_L_raw(i,3),26,count);
        [ankle_L(i,1),ankle_L(i,2),ankle_L(i,3),count] = TrackKalman3d(ankle_L_raw(i,1),ankle_L_raw(i,2),ankle_L_raw(i,3),27,count);
        [foot_L(i,1),foot_L(i,2),foot_L(i,3),count] = TrackKalman3d(foot_L_raw(i,1),foot_L_raw(i,2),foot_L_raw(i,3),28,count);


        [knee_R(i,1),knee_R(i,2),knee_R(i,3),count] = TrackKalman3d(knee_R_raw(i,1),knee_R_raw(i,2),knee_R_raw(i,3),29,count);
        [ankle_R(i,1),ankle_R(i,2),ankle_R(i,3),count] = TrackKalman3d(ankle_R_raw(i,1),ankle_R_raw(i,2),ankle_R_raw(i,3),30,count);
        [foot_R(i,1),foot_R(i,2),foot_R(i,3),count] = TrackKalman3d(foot_R_raw(i,1),foot_R_raw(i,2),foot_R_raw(i,3),31,count);

        [heel_L(i,1),heel_L(i,2),heel_L(i,3),count] = TrackKalman3d(heel_L_raw(i,1),heel_L_raw(i,2),heel_L_raw(i,3),32,count);
        [heel_R(i,1),heel_R(i,2),heel_R(i,3),count] = TrackKalman3d(heel_R_raw(i,1),heel_R_raw(i,2),heel_R_raw(i,3),33,count);

    end
    %% if (Kalman filter 미적용)

else

    pelvis = pelvis_raw;

    hip_L = hip_L_raw;
    hip_R = hip_R_raw;


    shoulder_L = shoulder_L_raw;
    elbow_L = elbow_L_raw;
    wrist_L = wrist_L_raw;
    %     hand_L = hand_L_raw;


    shoulder_R = shoulder_R_raw;
    elbow_R = elbow_R_raw;
    wrist_R = wrist_R_raw;
    %     hand_R = hand_R_raw;

    nose = nose_raw;
    eye_L = eye_L_raw;
    eye_R = eye_R_raw;
    ear_L = ear_L_raw;
    ear_R = ear_R_raw;

    knee_L = knee_L_raw;
    ankle_L = ankle_L_raw;
    foot_L = foot_L_raw;

    knee_R = knee_R_raw;
    ankle_R = ankle_R_raw;
    foot_R = foot_R_raw;

    heel_L=heel_L_raw;
    heel_R=heel_R_raw;



end

%% 2.3 Kalman filter 전후 Plot
% if kalmanplot_onoff == 1
% 
%     plot(time,ankle_R_raw(:,1),time,ankle_R(:,1),'linewidth',2) % Kalman Filter 전후 차이 비교
%     xlabel('time [sec]'); ylabel('position [m]')
%     legend('Rraw','Rkf')
%     xlim([0 20])
%     grid on
% 
%     figure()
%     plot(time,ankle_L_raw(:,1),time,ankle_L(:,1),'linewidth',2) % Kalman Filter 전후 차이 비교
%     xlabel('time [sec]'); ylabel('position [m]')
%     legend('Lraw','Lkf')
%     xlim([0 20])
%     grid on
% 
% else
% end

%% 2.4 Cut data (after Kalman Filter)

time = time(te:end,:);
md = length(time);

pelvis = pelvis(te:end,:);

hip_L = hip_L(te:end,:);
hip_R = hip_R(te:end,:);

shoulder_L = shoulder_L(te:end,:);
elbow_L = elbow_L(te:end,:);
wrist_L = wrist_L(te:end,:);

shoulder_R = shoulder_R(te:end,:);
elbow_R = elbow_R(te:end,:);
wrist_R = wrist_R(te:end,:);

nose = nose(te:end,:);
eye_L = eye_L(te:end,:);
eye_R = eye_R(te:end,:);
ear_L = ear_L(te:end,:);
ear_R = ear_R(te:end,:);

knee_L = knee_L(te:end,:);
ankle_L = ankle_L(te:end,:);
foot_L = foot_L(te:end,:);

knee_R = knee_R(te:end,:);
ankle_R = ankle_R(te:end,:);
foot_R = foot_R(te:end,:);

heel_L = heel_L(te:end,:);
heel_R = heel_R(te:end,:);

%% 3 Low-pass Filter (LPF)

%% 3.1 Data resampling (50Hz)
if LPF_onoff==1

    time_forLPF=(time(1):0.02:time(end))';
    for i=1:3
        hip_L_resampling(:,i) = spline(time,hip_L(:,i),time_forLPF);
        hip_R_resampling(:,i) = spline(time,hip_R(:,i),time_forLPF);

        shoulder_L_resampling(:,i) = spline(time,shoulder_L(:,i),time_forLPF);
        shoulder_R_resampling(:,i) = spline(time,shoulder_R(:,i),time_forLPF);

        elbow_L_resampling(:,i) = spline(time,elbow_L(:,i),time_forLPF);
        elbow_R_resampling(:,i) = spline(time,elbow_R(:,i),time_forLPF);

        knee_L_resampling(:,i) = spline(time,knee_L(:,i),time_forLPF);
        knee_R_resampling(:,i) = spline(time,knee_R(:,i),time_forLPF);

        ankle_L_resampling(:,i) = spline(time,ankle_L(:,i),time_forLPF);
        ankle_R_resampling(:,i) = spline(time,ankle_R(:,i),time_forLPF);

        heel_L_resampling(:,i) = spline(time,heel_L(:,i),time_forLPF);
        heel_R_resampling(:,i) = spline(time,heel_R(:,i),time_forLPF);

        foot_L_resampling(:,i) = spline(time,foot_L(:,i),time_forLPF);
        foot_R_resampling(:,i) = spline(time,foot_R(:,i),time_forLPF);


    end



    %% 3.2 FFT 및 LPF 적용, Plot

    if HSTO_landmark =='A'

        ankle_L_X_forHSTO = spline(time,ankle_L(:,1),time_forLPF);
        ankle_R_X_forHSTO = spline(time,ankle_R(:,1),time_forLPF);

        fs=50;
        T=1/fs;
        L=length(ankle_L_X_forHSTO);
        t=(0:L-1)*T;

        YLb=fft(ankle_L_X_forHSTO);
        P2Lb=abs(YLb/L);
        P1Lb=P2Lb(1:L/2+1);
        P1Lb(2:end-1)=2*P1Lb(2:end-1);

        YRb=fft(ankle_R_X_forHSTO);
        P2Rb=abs(YRb/L);
        P1Rb=P2Rb(1:L/2+1);
        P1Rb(2:end-1)=2*P1Rb(2:end-1);

        f=fs*(0:(L/2))/L;

        ankle_L_X_forHSTO=lowpass(ankle_L_X_forHSTO, LPF_fq, 50,'Steepness',0.99);
        ankle_R_X_forHSTO=lowpass(ankle_R_X_forHSTO, LPF_fq, 50,'Steepness',0.99);

        YL=fft(ankle_L_X_forHSTO);
        P2L=abs(YL/L);
        P1L=P2L(1:L/2+1);
        P1L(2:end-1)=2*P1L(2:end-1);

        YR=fft(ankle_R_X_forHSTO);
        P2R=abs(YR/L);
        P1R=P2R(1:L/2+1);
        P1R(2:end-1)=2*P1R(2:end-1);
%         if fft_plot_onoff==1
%             figure()
%             subplot(2,1,1)
%             plot(f,P1Lb,'r',f,P1L,'--r');
%             hold on
%             title('ankle L LPF FFT')
%             subplot(2,1,2)
%             plot(f,P1Rb,'b',f,P1R,'--b');
%             hold on
%             title('ankle R LPF FFT')
% 
%             figure()
%             subplot(2,1,1)
%             plot(time,ankle_L(:,1),'r',time_forLPF,ankle_L_X_forHSTO,'--r');
%             hold on
%             title('ankle L LPF')
%             subplot(2,1,1)
%             plot(time,ankle_R(:,1),'b',time_forLPF,ankle_R_X_forHSTO,'--b');
%             hold on
%             title('ankle R LPF')
%         end
         L_HSTO=ankle_L_X_forHSTO;
         R_HSTO=ankle_R_X_forHSTO;
% 
%     elseif HSTO_landmark=='H'
%         heel_L_X_forHSTO = spline(time,heel_L(:,1),time_forLPF);
%         heel_R_X_forHSTO = spline(time,heel_R(:,1),time_forLPF);
% 
%         fs=50;
%         T=1/fs;
%         L=length(heel_L_X_forHSTO);
%         t=(0:L-1)*T;
% 
%         YLb=fft(heel_L_X_forHSTO);
%         P2Lb=abs(YLb/L);
%         P1Lb=P2Lb(1:L/2+1);
%         P1Lb(2:end-1)=2*P1Lb(2:end-1);
% 
%         YRb=fft(heel_R_X_forHSTO);
%         P2Rb=abs(YRb/L);
%         P1Rb=P2Rb(1:L/2+1);
%         P1Rb(2:end-1)=2*P1Rb(2:end-1);
% 
%         f=fs*(0:(L/2))/L;
% 
%         heel_L_X_forHSTO=lowpass(heel_L_X_forHSTO, LPF_fq, 50,'Steepness',0.99);
%         heel_R_X_forHSTO=lowpass(heel_R_X_forHSTO, LPF_fq, 50,'Steepness',0.99);
% 
%         YL=fft(heel_L_X_forHSTO);
%         P2L=abs(YL/L);
%         P1L=P2L(1:L/2+1);
%         P1L(2:end-1)=2*P1L(2:end-1);
% 
%         YR=fft(heel_R_X_forHSTO);
%         P2R=abs(YR/L);
%         P1R=P2R(1:L/2+1);
%         P1R(2:end-1)=2*P1R(2:end-1);
%         if fft_plot_onoff==1
%             figure()
%             
%             plot(f,P1Lb,'r',f,P1L,'--r');
%             hold on
%             title('heel L LPF FFT')
%             
%             plot(f,P1Rb,'b',f,P1R,'--b');
%             hold on
%             
% 
%             figure()
%             
%             plot(time,heel_L(:,1),'r',time_forLPF,heel_L_X_forHSTO,'--r');
%             hold on
%             title('heel L LPF')
%             
%             plot(time,heel_R(:,1),'b',time_forLPF,heel_R_X_forHSTO,'--b');
%             hold on
%             
%         end
%         L_HSTO=heel_L_X_forHSTO;
%         R_HSTO=heel_R_X_forHSTO;
% 
%     elseif HSTO_landmark=='F'
%         foot_L_X_forHSTO = spline(time,foot_L(:,1),time_forLPF);
%         foot_R_X_forHSTO = spline(time,foot_R(:,1),time_forLPF);
% 
%         fs=50;
%         T=1/fs;
%         L=length(foot_L_X_forHSTO);
%         t=(0:L-1)*T;
% 
%         YLb=fft(foot_L_X_forHSTO);
%         P2Lb=abs(YLb/L);
%         P1Lb=P2Lb(1:L/2+1);
%         P1Lb(2:end-1)=2*P1Lb(2:end-1);
% 
%         YRb=fft(foot_R_X_forHSTO);
%         P2Rb=abs(YRb/L);
%         P1Rb=P2Rb(1:L/2+1);
%         P1Rb(2:end-1)=2*P1Rb(2:end-1);
% 
%         f=fs*(0:(L/2))/L;
% 
%         foot_L_X_forHSTO=lowpass(foot_L_X_forHSTO, LPF_fq, 50,'Steepness',0.99);
%         foot_R_X_forHSTO=lowpass(foot_R_X_forHSTO, LPF_fq, 50,'Steepness',0.99);
% 
%         YL=fft(foot_L_X_forHSTO);
%         P2L=abs(YL/L);
%         P1L=P2L(1:L/2+1);
%         P1L(2:end-1)=2*P1L(2:end-1);
% 
%         YR=fft(foot_R_X_forHSTO);
%         P2R=abs(YR/L);
%         P1R=P2R(1:L/2+1);
%         P1R(2:end-1)=2*P1R(2:end-1);
%         if fft_plot_onoff==1
%             figure()
%             subplot(2,1,1)
%             plot(f,P1Lb,'r',f,P1L,'--r');
%             hold on
%             title('foot L LPF FFT')
%             subplot(2,1,2)
%             plot(f,P1Rb,'b',f,P1R,'--b');
%             hold on
%             title('foot R LPF FFT')
% 
%             figure()
%             subplot(2,1,1)
%             plot(time,foot_L(:,1),'r',time_forLPF,foot_L_X_forHSTO,'--r');
%             hold on
%             title('foot L LPF')
%             subplot(2,1,1)
%             plot(time,foot_R(:,1),'b',time_forLPF,foot_R_X_forHSTO,'--b');
%             hold on
%             title('foot R LPF')
%         end
%         L_HSTO=foot_L_X_forHSTO;
%         R_HSTO=foot_R_X_forHSTO;
% 
     end



    %% 3.3 Resampling Data 로 변환
    time=time_forLPF;

    hip_L=hip_L_resampling;
    hip_R=hip_R_resampling;

    shoulder_L=shoulder_L_resampling;
    shoulder_R=shoulder_R_resampling;

    elbow_L=elbow_L_resampling;
    elbow_R=elbow_R_resampling;

    knee_L=knee_L_resampling;
    knee_R=knee_R_resampling;

    ankle_L=ankle_L_resampling;
    ankle_R=ankle_R_resampling;

    heel_L=heel_L_resampling;
    heel_R=heel_R_resampling;

    foot_L=foot_L_resampling;
    foot_R=foot_R_resampling;

    %% 3.4 LPF 미적용시

else
    if HSTO_landmark=='A'
        L_HSTO=ankle_L(:,1);
        R_HSTO=ankle_R(:,1);
    elseif HSTO_landmark=='F'
        L_HSTO=foot_L(:,1);
        R_HSTO=foot_R(:,1);
    elseif HSTO_landmark=='H'
        L_HSTO=heel_L(:,1);
        R_HSTO=heel_R(:,1);
    end
end


%% 4 Stride decomposition

%% 4.1 HSTO check
% 오른발 기준으로 계산

if gaitfeatures_onoff == 1

    %% 4.1.1 분석 구간 추출

    wp_mps = wp_kph*wp_gain/3.6; % kph to mps

    num_gaitstart = find(time<=sec_gaitstart);
    num_gaitstart = num_gaitstart(end);
    num_gaitend = find(time<=sec_gaitend);
    num_gaitend = num_gaitend(end);

    time_gait = time(num_gaitstart:num_gaitend,:);
    hip_L_gait = hip_L(num_gaitstart:num_gaitend,:);
    hip_R_gait = hip_R(num_gaitstart:num_gaitend,:);
    knee_L_gait = knee_L(num_gaitstart:num_gaitend,:);
    knee_R_gait = knee_R(num_gaitstart:num_gaitend,:);
    ankle_L_gait = ankle_L(num_gaitstart:num_gaitend,:);
    ankle_R_gait = ankle_R(num_gaitstart:num_gaitend,:);
    foot_L_gait = foot_L(num_gaitstart:num_gaitend,:);
    foot_R_gait = foot_R(num_gaitstart:num_gaitend,:);
    heel_L_gait = heel_L(num_gaitstart:num_gaitend,:);
    heel_R_gait = heel_R(num_gaitstart:num_gaitend,:);
    shoulder_L_gait = shoulder_L(num_gaitstart:num_gaitend,:);
    shoulder_R_gait = shoulder_R(num_gaitstart:num_gaitend,:);
    elbow_L_gait = elbow_L(num_gaitstart:num_gaitend,:);
    elbow_R_gait = elbow_R(num_gaitstart:num_gaitend,:);

    L_HSTO_gait = L_HSTO(num_gaitstart:num_gaitend,:);
    R_HSTO_gait = R_HSTO(num_gaitstart:num_gaitend,:);

    %% 4.1.2 1차 HSTO check
    [pks_L_HS1,crit_L_HS1] = findpeaks(L_HSTO_gait(:,1),'MinPeakDistance',MPD); %#ok<*ASGLU>
    [pks_L_TO1,crit_L_TO1] = findpeaks(-L_HSTO_gait(:,1),'MinPeakDistance',MPD);
    [pks_R_HS1,crit_R_HS1] = findpeaks(R_HSTO_gait(:,1),'MinPeakDistance',MPD);
    [pks_R_TO1,crit_R_TO1] = findpeaks(-R_HSTO_gait(:,1),'MinPeakDistance',MPD);

    %     MPD_gain = 0.7;
    %% 4.1.3 2차 HSTO check
    [pks_L_HS,crit_L_HS] = findpeaks(L_HSTO_gait(:,1),'MinPeakDistance',MPD_gain*mean(diff(crit_L_HS1)));
    [pks_L_TO,crit_L_TO] = findpeaks(-L_HSTO_gait(:,1),'MinPeakDistance',MPD_gain*mean(diff(crit_L_TO1)));
    [pks_R_HS,crit_R_HS] = findpeaks(R_HSTO_gait(:,1),'MinPeakDistance',MPD_gain*mean(diff(crit_R_HS1)));
    [pks_R_TO,crit_R_TO] = findpeaks(-R_HSTO_gait(:,1),'MinPeakDistance',MPD_gain*mean(diff(crit_R_TO1)));


    %% 4.1.4 HSTO 정리
    % heel strike가 toe off보다 먼저 나오고, toe off보다 heel strike가 나중에 나오게 정리 (heel to heel = 1 cycle)

    if crit_L_TO(1) < crit_L_HS(1)
        crit_L_TO = crit_L_TO(2:end);
        pks_L_TO = pks_L_TO(2:end);
    end

    if crit_L_HS(end) < crit_L_TO(end)
        crit_L_TO = crit_L_TO(1:end-1);
        pks_L_TO = pks_L_TO(1:end-1);
    end

    if crit_R_TO(1) < crit_R_HS(1)
        crit_R_TO = crit_R_TO(2:end);
        pks_R_TO = pks_R_TO(2:end);
    end

    if crit_R_HS(end) < crit_R_TO(end)
        crit_R_TO = crit_R_TO(1:end-1);
        pks_R_TO = pks_R_TO(1:end-1);
    end

    %% cadence [stride/min]

    for g = 2:length(crit_L_HS)
        H2H_L(g-1,1) = 60/(time_gait(crit_L_HS(g))-time_gait(crit_L_HS(g-1)));
    end
    H2H_m = mean(H2H_L);
    H2H_std = std(H2H_L);
    GAIT_cadence_L_cpm = H2H_m;
    GAIT_cadence_L_cpm_std = H2H_std;


    for g = 2:length(crit_R_HS)
        H2H_R(g-1,1) = 60/(time_gait(crit_R_HS(g))-time_gait(crit_R_HS(g-1)));
    end
    H2H_m = mean(H2H_R);
    H2H_std = std(H2H_R);
    GAIT_cadence_R_cpm = H2H_m;
    GAIT_cadence_R_cpm_std = H2H_std;

    GAIT_cadence_A_cpm = round(mean([GAIT_cadence_L_cpm GAIT_cadence_R_cpm]),1);
    GAIT_cadence_A_cpm_std = round(mean([GAIT_cadence_L_cpm_std GAIT_cadence_R_cpm_std]),1);

    %% 4.1.5 HSTO plot
%     if HSTOcheck_onoff == 1
% 
%         figure
%         subplot(2,1,1)
%         plot(time_gait,L_HSTO_gait(:,1))
%         title(strcat('X Coordinate of Left',' ',HSTO_landmark)); xlabel('Time (sec)'); ylabel('Position (m)')
%         grid on
%         hold on
%         plot(time_gait(crit_L_HS),pks_L_HS,'or')
%         plot(time_gait(crit_L_TO),-pks_L_TO,'ob')
%         plot(time_gait(crit_L_HS1),pks_L_HS1,'+r')
%         plot(time_gait(crit_L_TO1),-pks_L_TO1,'+b')
%         hold off
% 
%         subplot(2,1,2)
%         plot(time_gait,R_HSTO_gait(:,1))
%         title(strcat('X Coordinate of Right',' ',HSTO_landmark))
%         grid on
%         hold on
%         plot(time_gait(crit_R_HS),pks_R_HS,'or')
%         plot(time_gait(crit_R_TO),-pks_R_TO,'ob')
%         plot(time_gait(crit_R_HS1),pks_R_HS1,'+r')
%         plot(time_gait(crit_R_TO1),-pks_R_TO1,'+b')
%         hold off
% 
%     else
%     end

else
end


%% 4.2 Joint angle calculation (at time axis)

%% Trunk angle calculation
Trunk_R_angle_Y=rad2deg(atan((shoulder_R_gait(:,1)-hip_R_gait(:,1))./(shoulder_R_gait(:,3)-hip_R_gait(:,3))));
Trunk_L_angle_Y=rad2deg(atan((shoulder_L_gait(:,1)-hip_L_gait(:,1))./(shoulder_L_gait(:,3)-hip_L_gait(:,3))));

shoulder_angle_Z=-rad2deg(atan((shoulder_R_gait(:,1)-shoulder_L_gait(:,1))./(shoulder_R_gait(:,2)-shoulder_L_gait(:,2))));
pelvic_angle_Z=-rad2deg(atan((hip_R_gait(:,1)-hip_L_gait(:,1))./(hip_R_gait(:,2)-hip_L_gait(:,2))));

shoulder_angle_X=-rad2deg(atan((shoulder_L_gait(:,3)-shoulder_R_gait(:,3))./(shoulder_L_gait(:,2)-shoulder_R_gait(:,2))));
pelvic_angle_X=-rad2deg(atan((hip_L_gait(:,3)-hip_R_gait(:,3))./(hip_L_gait(:,2)-hip_R_gait(:,2))));


%% UA angle calculation
UA_R_angle_Y=rad2deg(atan((shoulder_R_gait(:,1)-elbow_R_gait(:,1))./(shoulder_R_gait(:,3)-elbow_R_gait(:,3))));
UA_L_angle_Y=rad2deg(atan((shoulder_L_gait(:,1)-elbow_L_gait(:,1))./(shoulder_L_gait(:,3)-elbow_L_gait(:,3))));

%% UL angle calculation

UL_R_angle_X=-rad2deg(atan((hip_R_gait(:,2)-knee_R_gait(:,2))./(hip_R_gait(:,3)-knee_R_gait(:,3))));
UL_L_angle_X=-rad2deg(atan((hip_L_gait(:,2)-knee_L_gait(:,2))./(hip_L_gait(:,3)-knee_L_gait(:,3))));

UL_R_angle_Y=rad2deg(atan((hip_R_gait(:,1)-knee_R_gait(:,1))./(hip_R_gait(:,3)-knee_R_gait(:,3))));
UL_L_angle_Y=rad2deg(atan((hip_L_gait(:,1)-knee_L_gait(:,1))./(hip_L_gait(:,3)-knee_L_gait(:,3))));

%% LL angle calculation
LL_L_angle_X=-rad2deg(atan((knee_L_gait(:,2)-ankle_L_gait(:,2))./(knee_L_gait(:,3)-ankle_L_gait(:,3))));
LL_R_angle_X=-rad2deg(atan((knee_R_gait(:,2)-ankle_R_gait(:,2))./(knee_R_gait(:,3)-ankle_R_gait(:,3))));

LL_L_angle_Y=rad2deg(atan((knee_L_gait(:,1)-ankle_L_gait(:,1))./(knee_L_gait(:,3)-ankle_L_gait(:,3))));
LL_R_angle_Y=rad2deg(atan((knee_R_gait(:,1)-ankle_R_gait(:,1))./(knee_R_gait(:,3)-ankle_R_gait(:,3))));

%% Foot angle calculation

foot_R_angle_Y=rad2deg(atan((heel_R_gait(:,3)-foot_R_gait(:,3))./(heel_R_gait(:,1)-foot_R_gait(:,1))));
foot_L_angle_Y=rad2deg(atan((heel_L_gait(:,3)-foot_L_gait(:,3))./(heel_L_gait(:,1)-foot_L_gait(:,1))));

%% Shoulder joint angle
shoulder_R_angle_Y=Trunk_R_angle_Y-UA_R_angle_Y;
shoulder_L_angle_Y=Trunk_L_angle_Y-UA_L_angle_Y;

%% Hip joint angle
hip_R_angle_Y=Trunk_R_angle_Y-UL_R_angle_Y;
hip_L_angle_Y=Trunk_L_angle_Y-UL_L_angle_Y;



%% Knee joint angle calculation
knee_R_angle_X=UL_R_angle_X-LL_R_angle_X;
knee_L_angle_X=-UL_L_angle_X+LL_L_angle_X;

knee_R_angle_Y=-UL_R_angle_Y+LL_R_angle_Y;
knee_L_angle_Y=-UL_L_angle_Y+LL_L_angle_Y;


%% Ankle joint angle calculation
ankle_R_angle_Y=foot_R_angle_Y+LL_R_angle_Y;
ankle_L_angle_Y=foot_L_angle_Y+LL_L_angle_Y;

%% COG Lateral calculation
COG_left=shoulder_L_gait(:,2)-heel_L_gait(:,2) ...
    + hip_L_gait(:,2)-heel_L_gait(:,2);
COG_right=-(shoulder_R_gait(:,2)-heel_R_gait(:,2) ...
    + hip_R_gait(:,2)-heel_R_gait(:,2));


%% 4.3 Resampling (gait pct)

samptime = [0:0.02:1]';
Left_header={'%cycle','mean','mean-sd','mean+sd'};
Right_header={'%cycle','mean','mean-sd','mean+sd'};
%% 4.3.1 Right Heel strike 기준 resampling
for i = 1:length(crit_R_HS)-1
    per = (time_gait(crit_R_HS(i):crit_R_HS(i+1))-time_gait(crit_R_HS(i)))/(time_gait(crit_R_HS(i+1))-time_gait(crit_R_HS(i)));

    in = shoulder_R_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        shoulder_R_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = shoulder_R_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        shoulder_R_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = hip_R_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        hip_R_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = hip_R_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        hip_R_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = knee_R_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        knee_R_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = knee_R_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        knee_R_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = ankle_R_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        ankle_R_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = ankle_R_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        ankle_R_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = heel_R_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        heel_R_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = heel_R_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        heel_R_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = shoulder_L_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        shoulder_RL_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = shoulder_L_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        shoulder_RL_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = hip_L_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        hip_RL_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = hip_L_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        hip_RL_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = knee_L_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        knee_RL_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = knee_L_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        knee_RL_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = ankle_L_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        ankle_RL_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = ankle_L_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        ankle_RL_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = heel_L_gait(crit_R_HS(i):crit_R_HS(i+1),2); ...
        heel_RL_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = heel_L_gait(crit_R_HS(i):crit_R_HS(i+1),3); ...
        heel_RL_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = knee_R_angle_X(crit_R_HS(i):crit_R_HS(i+1)); ...
        knee_R_angle_X_pct(:,i) = spline(per,in,samptime);

    in = knee_R_angle_Y(crit_R_HS(i):crit_R_HS(i+1)); ...
        knee_R_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = hip_R_angle_Y(crit_R_HS(i):crit_R_HS(i+1)); ...
        hip_R_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = ankle_R_angle_Y(crit_R_HS(i):crit_R_HS(i+1)); ...
        ankle_R_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = shoulder_angle_X(crit_R_HS(i):crit_R_HS(i+1)); ...
        shoulder_angle_X_pct(:,i) = spline(per,in,samptime);

    in = pelvic_angle_X(crit_R_HS(i):crit_R_HS(i+1)); ...
        pelvic_angle_X_pct(:,i) = spline(per,in,samptime);

    in = shoulder_angle_Z(crit_R_HS(i):crit_R_HS(i+1)); ...
        shoulder_angle_Z_pct(:,i) = spline(per,in,samptime);

    in = pelvic_angle_Z(crit_R_HS(i):crit_R_HS(i+1)); ...
        pelvic_angle_Z_pct(:,i) = spline(per,in,samptime);

    in = shoulder_R_angle_Y(crit_R_HS(i):crit_R_HS(i+1)); ...
        shoulder_R_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = Trunk_R_angle_Y(crit_R_HS(i):crit_R_HS(i+1)); ...
        Trunk_R_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = COG_right(crit_R_HS(i):crit_R_HS(i+1)); ...
        COG_Lateral_R_pct(:,i) = spline(per,in,samptime);


    stand_R_pct(i)=(time_gait(crit_R_TO(i))-time_gait(crit_R_HS(i)))/(time_gait(crit_R_HS(i+1))-time_gait(crit_R_HS(i)));
    Right_header=[Right_header,strcat('gait#',num2str(i))];

end

%% 4.3.2 Left Heel strike 기준 resampling
for i = 1:length(crit_L_HS)-1
    per = (time_gait(crit_L_HS(i):crit_L_HS(i+1))-time_gait(crit_L_HS(i)))/(time_gait(crit_L_HS(i+1))-time_gait(crit_L_HS(i)));

    in = shoulder_L_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        shoulder_L_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = shoulder_L_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        shoulder_L_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = hip_L_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        hip_L_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = hip_L_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        hip_L_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = knee_L_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        knee_L_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = knee_L_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        knee_L_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = ankle_L_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        ankle_L_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = ankle_L_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        ankle_L_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = heel_L_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        heel_L_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = heel_L_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        heel_L_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = shoulder_R_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        shoulder_LR_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = shoulder_R_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        shoulder_LR_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = hip_R_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        hip_LR_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = hip_R_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        hip_LR_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = knee_R_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        knee_LR_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = knee_R_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        knee_LR_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = ankle_R_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        ankle_LR_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = ankle_R_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        ankle_LR_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = heel_R_gait(crit_L_HS(i):crit_L_HS(i+1),2); ...
        heel_LR_gait_Y_pct(:,i) = spline(per,in,samptime);
    in = heel_R_gait(crit_L_HS(i):crit_L_HS(i+1),3); ...
        heel_LR_gait_Z_pct(:,i) = spline(per,in,samptime);

    in = hip_L_angle_Y(crit_L_HS(i):crit_L_HS(i+1)); ...
        hip_L_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = ankle_L_angle_Y(crit_L_HS(i):crit_L_HS(i+1)); ...
        ankle_L_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = knee_L_angle_X(crit_L_HS(i):crit_L_HS(i+1)); ...
        knee_L_angle_X_pct(:,i) = spline(per,in,samptime);
    in = knee_L_angle_Y(crit_L_HS(i):crit_L_HS(i+1)); ...
        knee_L_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = shoulder_L_angle_Y(crit_L_HS(i):crit_L_HS(i+1)); ...
        shoulder_L_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = Trunk_L_angle_Y(crit_L_HS(i):crit_L_HS(i+1)); ...
        Trunk_L_angle_Y_pct(:,i) = spline(per,in,samptime);

    in = COG_left(crit_L_HS(i):crit_L_HS(i+1)); ...
        COG_Lateral_L_pct(:,i) = spline(per,in,samptime);



    stand_L_pct(i)=(time_gait(crit_L_TO(i))-time_gait(crit_L_HS(i)))/(time_gait(crit_L_HS(i+1))-time_gait(crit_L_HS(i)));
    Left_header=[Left_header,strcat('gait#',num2str(i))];
end
COG_Lateral_L=(COG_Lateral_L_pct(16,1:length(crit_R_HS)-3)./(COG_Lateral_L_pct(16,1:length(crit_R_HS)-3)+COG_Lateral_R_pct(16,1:length(crit_R_HS)-3))*100);
COG_Lateral_R=(COG_Lateral_R_pct(16,1:length(crit_R_HS)-3)./(COG_Lateral_L_pct(16,1:length(crit_R_HS)-3)+COG_Lateral_R_pct(16,1:length(crit_R_HS)-3))*100);
COG_Lateral_L_pct_mstd=[mean(COG_Lateral_L) std(COG_Lateral_L)];
COG_Lateral_R_pct_mstd=[mean(COG_Lateral_R) std(COG_Lateral_R)];
%% 4.4 mstd 계산
for i=1:length(samptime)
    shoulder_L_gait_Y_pct_mstd(i,1:3)=[mean(shoulder_L_gait_Y_pct(i,:)) mean(shoulder_L_gait_Y_pct(i,:))-std(shoulder_L_gait_Y_pct(i,:)) mean(shoulder_L_gait_Y_pct(i,:))+std(shoulder_L_gait_Y_pct(i,:))];
    shoulder_L_gait_Z_pct_mstd(i,1:3)=[mean(shoulder_L_gait_Z_pct(i,:)) mean(shoulder_L_gait_Z_pct(i,:))-std(shoulder_L_gait_Z_pct(i,:)) mean(shoulder_L_gait_Z_pct(i,:))+std(shoulder_L_gait_Z_pct(i,:))];

    shoulder_R_gait_Y_pct_mstd(i,1:3)=[mean(shoulder_R_gait_Y_pct(i,:)) mean(shoulder_R_gait_Y_pct(i,:))-std(shoulder_R_gait_Y_pct(i,:)) mean(shoulder_R_gait_Y_pct(i,:))+std(shoulder_R_gait_Y_pct(i,:))];
    shoulder_R_gait_Z_pct_mstd(i,1:3)=[mean(shoulder_R_gait_Z_pct(i,:)) mean(shoulder_R_gait_Z_pct(i,:))-std(shoulder_R_gait_Z_pct(i,:)) mean(shoulder_R_gait_Z_pct(i,:))+std(shoulder_R_gait_Z_pct(i,:))];

    hip_L_gait_Y_pct_mstd(i,1:3)=[mean(hip_L_gait_Y_pct(i,:)) mean(hip_L_gait_Y_pct(i,:))-std(hip_L_gait_Y_pct(i,:)) mean(hip_L_gait_Y_pct(i,:))+std(hip_L_gait_Y_pct(i,:))];
    hip_L_gait_Z_pct_mstd(i,1:3)=[mean(hip_L_gait_Z_pct(i,:)) mean(hip_L_gait_Z_pct(i,:))-std(hip_L_gait_Z_pct(i,:)) mean(hip_L_gait_Z_pct(i,:))+std(hip_L_gait_Z_pct(i,:))];

    hip_R_gait_Y_pct_mstd(i,1:3)=[mean(hip_R_gait_Y_pct(i,:)) mean(hip_R_gait_Y_pct(i,:))-std(hip_R_gait_Y_pct(i,:)) mean(hip_R_gait_Y_pct(i,:))+std(hip_R_gait_Y_pct(i,:))];
    hip_R_gait_Z_pct_mstd(i,1:3)=[mean(hip_R_gait_Z_pct(i,:)) mean(hip_R_gait_Z_pct(i,:))-std(hip_R_gait_Z_pct(i,:)) mean(hip_R_gait_Z_pct(i,:))+std(hip_R_gait_Z_pct(i,:))];

    knee_L_gait_Y_pct_mstd(i,1:3)=[mean(knee_L_gait_Y_pct(i,:)) mean(knee_L_gait_Y_pct(i,:))-std(knee_L_gait_Y_pct(i,:)) mean(knee_L_gait_Y_pct(i,:))+std(knee_L_gait_Y_pct(i,:))];
    knee_L_gait_Z_pct_mstd(i,1:3)=[mean(knee_L_gait_Z_pct(i,:)) mean(knee_L_gait_Z_pct(i,:))-std(knee_L_gait_Z_pct(i,:)) mean(knee_L_gait_Z_pct(i,:))+std(knee_L_gait_Z_pct(i,:))];

    knee_R_gait_Y_pct_mstd(i,1:3)=[mean(knee_R_gait_Y_pct(i,:)) mean(knee_R_gait_Y_pct(i,:))-std(knee_R_gait_Y_pct(i,:)) mean(knee_R_gait_Y_pct(i,:))+std(knee_R_gait_Y_pct(i,:))];
    knee_R_gait_Z_pct_mstd(i,1:3)=[mean(knee_R_gait_Z_pct(i,:)) mean(knee_R_gait_Z_pct(i,:))-std(knee_R_gait_Z_pct(i,:)) mean(knee_R_gait_Z_pct(i,:))+std(knee_R_gait_Z_pct(i,:))];

    ankle_L_gait_Y_pct_mstd(i,1:3)=[mean(ankle_L_gait_Y_pct(i,:)) mean(ankle_L_gait_Y_pct(i,:))-std(ankle_L_gait_Y_pct(i,:)) mean(ankle_L_gait_Y_pct(i,:))+std(ankle_L_gait_Y_pct(i,:))];
    ankle_L_gait_Z_pct_mstd(i,1:3)=[mean(ankle_L_gait_Z_pct(i,:)) mean(ankle_L_gait_Z_pct(i,:))-std(ankle_L_gait_Z_pct(i,:)) mean(ankle_L_gait_Z_pct(i,:))+std(ankle_L_gait_Z_pct(i,:))];

    ankle_R_gait_Y_pct_mstd(i,1:3)=[mean(ankle_R_gait_Y_pct(i,:)) mean(ankle_R_gait_Y_pct(i,:))-std(ankle_R_gait_Y_pct(i,:)) mean(ankle_R_gait_Y_pct(i,:))+std(ankle_R_gait_Y_pct(i,:))];
    ankle_R_gait_Z_pct_mstd(i,1:3)=[mean(ankle_R_gait_Z_pct(i,:)) mean(ankle_R_gait_Z_pct(i,:))-std(ankle_R_gait_Z_pct(i,:)) mean(ankle_R_gait_Z_pct(i,:))+std(ankle_R_gait_Z_pct(i,:))];

    heel_L_gait_Y_pct_mstd(i,1:3)=[mean(heel_L_gait_Y_pct(i,:)) mean(heel_L_gait_Y_pct(i,:))-std(heel_L_gait_Y_pct(i,:)) mean(heel_L_gait_Y_pct(i,:))+std(heel_L_gait_Y_pct(i,:))];
    heel_L_gait_Z_pct_mstd(i,1:3)=[mean(heel_L_gait_Z_pct(i,:)) mean(heel_L_gait_Z_pct(i,:))-std(heel_L_gait_Z_pct(i,:)) mean(heel_L_gait_Z_pct(i,:))+std(heel_L_gait_Z_pct(i,:))];

    heel_R_gait_Y_pct_mstd(i,1:3)=[mean(heel_R_gait_Y_pct(i,:)) mean(heel_R_gait_Y_pct(i,:))-std(heel_R_gait_Y_pct(i,:)) mean(heel_R_gait_Y_pct(i,:))+std(heel_R_gait_Y_pct(i,:))];
    heel_R_gait_Z_pct_mstd(i,1:3)=[mean(heel_R_gait_Z_pct(i,:)) mean(heel_R_gait_Z_pct(i,:))-std(heel_R_gait_Z_pct(i,:)) mean(heel_R_gait_Z_pct(i,:))+std(heel_R_gait_Z_pct(i,:))];

    shoulder_LR_gait_Y_pct_mstd(i,1:3)=[mean(shoulder_LR_gait_Y_pct(i,:)) mean(shoulder_LR_gait_Y_pct(i,:))-std(shoulder_LR_gait_Y_pct(i,:)) mean(shoulder_LR_gait_Y_pct(i,:))+std(shoulder_LR_gait_Y_pct(i,:))];
    shoulder_LR_gait_Z_pct_mstd(i,1:3)=[mean(shoulder_LR_gait_Z_pct(i,:)) mean(shoulder_LR_gait_Z_pct(i,:))-std(shoulder_LR_gait_Z_pct(i,:)) mean(shoulder_LR_gait_Z_pct(i,:))+std(shoulder_LR_gait_Z_pct(i,:))];

    shoulder_RL_gait_Y_pct_mstd(i,1:3)=[mean(shoulder_RL_gait_Y_pct(i,:)) mean(shoulder_RL_gait_Y_pct(i,:))-std(shoulder_RL_gait_Y_pct(i,:)) mean(shoulder_RL_gait_Y_pct(i,:))+std(shoulder_RL_gait_Y_pct(i,:))];
    shoulder_RL_gait_Z_pct_mstd(i,1:3)=[mean(shoulder_RL_gait_Z_pct(i,:)) mean(shoulder_RL_gait_Z_pct(i,:))-std(shoulder_RL_gait_Z_pct(i,:)) mean(shoulder_RL_gait_Z_pct(i,:))+std(shoulder_RL_gait_Z_pct(i,:))];

    hip_LR_gait_Y_pct_mstd(i,1:3)=[mean(hip_LR_gait_Y_pct(i,:)) mean(hip_LR_gait_Y_pct(i,:))-std(hip_LR_gait_Y_pct(i,:)) mean(hip_LR_gait_Y_pct(i,:))+std(hip_LR_gait_Y_pct(i,:))];
    hip_LR_gait_Z_pct_mstd(i,1:3)=[mean(hip_LR_gait_Z_pct(i,:)) mean(hip_LR_gait_Z_pct(i,:))-std(hip_LR_gait_Z_pct(i,:)) mean(hip_LR_gait_Z_pct(i,:))+std(hip_LR_gait_Z_pct(i,:))];

    hip_RL_gait_Y_pct_mstd(i,1:3)=[mean(hip_RL_gait_Y_pct(i,:)) mean(hip_RL_gait_Y_pct(i,:))-std(hip_RL_gait_Y_pct(i,:)) mean(hip_RL_gait_Y_pct(i,:))+std(hip_RL_gait_Y_pct(i,:))];
    hip_RL_gait_Z_pct_mstd(i,1:3)=[mean(hip_RL_gait_Z_pct(i,:)) mean(hip_RL_gait_Z_pct(i,:))-std(hip_RL_gait_Z_pct(i,:)) mean(hip_RL_gait_Z_pct(i,:))+std(hip_RL_gait_Z_pct(i,:))];

    knee_LR_gait_Y_pct_mstd(i,1:3)=[mean(knee_LR_gait_Y_pct(i,:)) mean(knee_LR_gait_Y_pct(i,:))-std(knee_LR_gait_Y_pct(i,:)) mean(knee_LR_gait_Y_pct(i,:))+std(knee_LR_gait_Y_pct(i,:))];
    knee_LR_gait_Z_pct_mstd(i,1:3)=[mean(knee_LR_gait_Z_pct(i,:)) mean(knee_LR_gait_Z_pct(i,:))-std(knee_LR_gait_Z_pct(i,:)) mean(knee_LR_gait_Z_pct(i,:))+std(knee_LR_gait_Z_pct(i,:))];

    knee_RL_gait_Y_pct_mstd(i,1:3)=[mean(knee_RL_gait_Y_pct(i,:)) mean(knee_RL_gait_Y_pct(i,:))-std(knee_RL_gait_Y_pct(i,:)) mean(knee_RL_gait_Y_pct(i,:))+std(knee_RL_gait_Y_pct(i,:))];
    knee_RL_gait_Z_pct_mstd(i,1:3)=[mean(knee_RL_gait_Z_pct(i,:)) mean(knee_RL_gait_Z_pct(i,:))-std(knee_RL_gait_Z_pct(i,:)) mean(knee_RL_gait_Z_pct(i,:))+std(knee_RL_gait_Z_pct(i,:))];

    ankle_LR_gait_Y_pct_mstd(i,1:3)=[mean(ankle_LR_gait_Y_pct(i,:)) mean(ankle_LR_gait_Y_pct(i,:))-std(ankle_LR_gait_Y_pct(i,:)) mean(ankle_LR_gait_Y_pct(i,:))+std(ankle_LR_gait_Y_pct(i,:))];
    ankle_LR_gait_Z_pct_mstd(i,1:3)=[mean(ankle_LR_gait_Z_pct(i,:)) mean(ankle_LR_gait_Z_pct(i,:))-std(ankle_LR_gait_Z_pct(i,:)) mean(ankle_LR_gait_Z_pct(i,:))+std(ankle_LR_gait_Z_pct(i,:))];

    ankle_RL_gait_Y_pct_mstd(i,1:3)=[mean(ankle_RL_gait_Y_pct(i,:)) mean(ankle_RL_gait_Y_pct(i,:))-std(ankle_RL_gait_Y_pct(i,:)) mean(ankle_RL_gait_Y_pct(i,:))+std(ankle_RL_gait_Y_pct(i,:))];
    ankle_RL_gait_Z_pct_mstd(i,1:3)=[mean(ankle_RL_gait_Z_pct(i,:)) mean(ankle_RL_gait_Z_pct(i,:))-std(ankle_RL_gait_Z_pct(i,:)) mean(ankle_RL_gait_Z_pct(i,:))+std(ankle_RL_gait_Z_pct(i,:))];

    heel_LR_gait_Y_pct_mstd(i,1:3)=[mean(heel_LR_gait_Y_pct(i,:)) mean(heel_LR_gait_Y_pct(i,:))-std(heel_LR_gait_Y_pct(i,:)) mean(heel_LR_gait_Y_pct(i,:))+std(heel_LR_gait_Y_pct(i,:))];
    heel_LR_gait_Z_pct_mstd(i,1:3)=[mean(heel_LR_gait_Z_pct(i,:)) mean(heel_LR_gait_Z_pct(i,:))-std(heel_LR_gait_Z_pct(i,:)) mean(heel_LR_gait_Z_pct(i,:))+std(heel_LR_gait_Z_pct(i,:))];

    heel_RL_gait_Y_pct_mstd(i,1:3)=[mean(heel_RL_gait_Y_pct(i,:)) mean(heel_RL_gait_Y_pct(i,:))-std(heel_RL_gait_Y_pct(i,:)) mean(heel_RL_gait_Y_pct(i,:))+std(heel_RL_gait_Y_pct(i,:))];
    heel_RL_gait_Z_pct_mstd(i,1:3)=[mean(heel_RL_gait_Z_pct(i,:)) mean(heel_RL_gait_Z_pct(i,:))-std(heel_RL_gait_Z_pct(i,:)) mean(heel_RL_gait_Z_pct(i,:))+std(heel_RL_gait_Z_pct(i,:))];

    hip_L_angle_Y_pct_mstd(i,1:4)=[mean(hip_L_angle_Y_pct(i,:)) mean(hip_L_angle_Y_pct(i,:))-std(hip_L_angle_Y_pct(i,:)) mean(hip_L_angle_Y_pct(i,:))+std(hip_L_angle_Y_pct(i,:)) std(hip_L_angle_Y_pct(i,:))];
    hip_R_angle_Y_pct_mstd(i,1:4)=[mean(hip_R_angle_Y_pct(i,:)) mean(hip_R_angle_Y_pct(i,:))-std(hip_R_angle_Y_pct(i,:)) mean(hip_R_angle_Y_pct(i,:))+std(hip_R_angle_Y_pct(i,:)) std(hip_R_angle_Y_pct(i,:))];

    knee_L_angle_X_pct_mstd(i,1:4)=[mean(knee_L_angle_X_pct(i,:)) mean(knee_L_angle_X_pct(i,:))-std(knee_L_angle_X_pct(i,:)) mean(knee_L_angle_X_pct(i,:))+std(knee_L_angle_X_pct(i,:)) std(knee_L_angle_X_pct(i,:))];
    knee_R_angle_X_pct_mstd(i,1:4)=[mean(knee_R_angle_X_pct(i,:)) mean(knee_R_angle_X_pct(i,:))-std(knee_R_angle_X_pct(i,:)) mean(knee_R_angle_X_pct(i,:))+std(knee_R_angle_X_pct(i,:)) std(knee_R_angle_X_pct(i,:))];

    knee_L_angle_Y_pct_mstd(i,1:4)=[mean(knee_L_angle_Y_pct(i,:)) mean(knee_L_angle_Y_pct(i,:))-std(knee_L_angle_Y_pct(i,:)) mean(knee_L_angle_Y_pct(i,:))+std(knee_L_angle_Y_pct(i,:)) std(knee_L_angle_Y_pct(i,:))];
    knee_R_angle_Y_pct_mstd(i,1:4)=[mean(knee_R_angle_Y_pct(i,:)) mean(knee_R_angle_Y_pct(i,:))-std(knee_R_angle_Y_pct(i,:)) mean(knee_R_angle_Y_pct(i,:))+std(knee_R_angle_Y_pct(i,:)) std(knee_R_angle_Y_pct(i,:))];

    ankle_L_angle_Y_pct_mstd(i,1:4)=[mean(ankle_L_angle_Y_pct(i,:)) mean(ankle_L_angle_Y_pct(i,:))-std(ankle_L_angle_Y_pct(i,:)) mean(ankle_L_angle_Y_pct(i,:))+std(ankle_L_angle_Y_pct(i,:)) std(ankle_L_angle_Y_pct(i,:))];
    ankle_R_angle_Y_pct_mstd(i,1:4)=[mean(ankle_R_angle_Y_pct(i,:)) mean(ankle_R_angle_Y_pct(i,:))-std(ankle_R_angle_Y_pct(i,:)) mean(ankle_R_angle_Y_pct(i,:))+std(ankle_R_angle_Y_pct(i,:)) std(ankle_R_angle_Y_pct(i,:))];

    pelvic_angle_X_pct_mstd(i,1:4)=[mean(pelvic_angle_X_pct(i,:)) mean(pelvic_angle_X_pct(i,:))-std(pelvic_angle_X_pct(i,:)) mean(pelvic_angle_X_pct(i,:))+std(pelvic_angle_X_pct(i,:)) std(pelvic_angle_X_pct(i,:))];
    shoulder_angle_X_pct_mstd(i,1:4)=[mean(shoulder_angle_X_pct(i,:)) mean(shoulder_angle_X_pct(i,:))-std(shoulder_angle_X_pct(i,:)) mean(shoulder_angle_X_pct(i,:))+std(shoulder_angle_X_pct(i,:)) std(shoulder_angle_X_pct(i,:))];

    pelvic_angle_Z_pct_mstd(i,1:4)=[mean(pelvic_angle_Z_pct(i,:)) mean(pelvic_angle_Z_pct(i,:))-std(pelvic_angle_Z_pct(i,:)) mean(pelvic_angle_Z_pct(i,:))+std(pelvic_angle_Z_pct(i,:)) std(pelvic_angle_Z_pct(i,:))];
    shoulder_angle_Z_pct_mstd(i,1:4)=[mean(shoulder_angle_Z_pct(i,:)) mean(shoulder_angle_Z_pct(i,:))-std(shoulder_angle_Z_pct(i,:)) mean(shoulder_angle_Z_pct(i,:))+std(shoulder_angle_Z_pct(i,:)) std(shoulder_angle_Z_pct(i,:))];

    shoulder_R_angle_Y_pct_mstd(i,1:4)=[mean(shoulder_R_angle_Y_pct(i,:)) mean(shoulder_R_angle_Y_pct(i,:))-std(shoulder_R_angle_Y_pct(i,:)) mean(shoulder_R_angle_Y_pct(i,:))+std(shoulder_R_angle_Y_pct(i,:)) std(shoulder_R_angle_Y_pct(i,:))];
    shoulder_L_angle_Y_pct_mstd(i,1:4)=[mean(shoulder_L_angle_Y_pct(i,:)) mean(shoulder_L_angle_Y_pct(i,:))-std(shoulder_L_angle_Y_pct(i,:)) mean(shoulder_L_angle_Y_pct(i,:))+std(shoulder_L_angle_Y_pct(i,:)) std(shoulder_L_angle_Y_pct(i,:))];

    Trunk_R_angle_Y_pct_mstd(i,1:4)=[mean(Trunk_R_angle_Y_pct(i,:)) mean(Trunk_R_angle_Y_pct(i,:))-std(Trunk_R_angle_Y_pct(i,:)) mean(Trunk_R_angle_Y_pct(i,:))+std(Trunk_R_angle_Y_pct(i,:)) std(Trunk_R_angle_Y_pct(i,:))];
    Trunk_L_angle_Y_pct_mstd(i,1:4)=[mean(Trunk_L_angle_Y_pct(i,:)) mean(Trunk_L_angle_Y_pct(i,:))-std(Trunk_L_angle_Y_pct(i,:)) mean(Trunk_L_angle_Y_pct(i,:))+std(Trunk_L_angle_Y_pct(i,:)) std(Trunk_L_angle_Y_pct(i,:))];

    

end
shoulder_L_gait_Y_pct_A = [samptime shoulder_L_gait_Y_pct_mstd(:,1:3) shoulder_L_gait_Y_pct];
shoulder_L_gait_Z_pct_A = [samptime shoulder_L_gait_Z_pct_mstd(:,1:3) shoulder_L_gait_Z_pct];

shoulder_R_gait_Y_pct_A = [samptime shoulder_R_gait_Y_pct_mstd(:,1:3) shoulder_R_gait_Y_pct];
shoulder_R_gait_Z_pct_A = [samptime shoulder_R_gait_Z_pct_mstd(:,1:3) shoulder_R_gait_Z_pct];

hip_L_gait_Y_pct_A = [samptime hip_L_gait_Y_pct_mstd(:,1:3) hip_L_gait_Y_pct];
hip_L_gait_Z_pct_A = [samptime hip_L_gait_Z_pct_mstd(:,1:3) hip_L_gait_Z_pct];

hip_R_gait_Y_pct_A = [samptime hip_R_gait_Y_pct_mstd hip_R_gait_Y_pct];
hip_R_gait_Z_pct_A = [samptime hip_R_gait_Z_pct_mstd hip_R_gait_Z_pct];

knee_L_gait_Y_pct_A = [samptime knee_L_gait_Y_pct_mstd knee_L_gait_Y_pct];
knee_L_gait_Z_pct_A = [samptime knee_L_gait_Z_pct_mstd knee_L_gait_Z_pct];

knee_R_gait_Y_pct_A = [samptime knee_R_gait_Y_pct_mstd knee_R_gait_Y_pct];
knee_R_gait_Z_pct_A = [samptime knee_R_gait_Z_pct_mstd knee_R_gait_Z_pct];

ankle_L_gait_Y_pct_A = [samptime ankle_L_gait_Y_pct_mstd ankle_L_gait_Y_pct];
ankle_L_gait_Z_pct_A = [samptime ankle_L_gait_Z_pct_mstd ankle_L_gait_Z_pct];

ankle_R_gait_Y_pct_A = [samptime ankle_R_gait_Y_pct_mstd ankle_R_gait_Y_pct];
ankle_R_gait_Z_pct_A = [samptime ankle_R_gait_Z_pct_mstd ankle_R_gait_Z_pct];

heel_L_gait_Y_pct_A = [samptime heel_L_gait_Y_pct_mstd heel_L_gait_Y_pct];
heel_L_gait_Z_pct_A = [samptime heel_L_gait_Z_pct_mstd heel_L_gait_Z_pct];

heel_R_gait_Y_pct_A = [samptime heel_R_gait_Y_pct_mstd heel_R_gait_Y_pct];
heel_R_gait_Z_pct_A = [samptime heel_R_gait_Z_pct_mstd heel_R_gait_Z_pct];

shoulder_LR_gait_Y_pct_A = [samptime shoulder_LR_gait_Y_pct_mstd shoulder_LR_gait_Y_pct];
shoulder_LR_gait_Z_pct_A = [samptime shoulder_LR_gait_Z_pct_mstd shoulder_LR_gait_Z_pct];

shoulder_RL_gait_Y_pct_A = [samptime shoulder_RL_gait_Y_pct_mstd shoulder_RL_gait_Y_pct];
shoulder_RL_gait_Z_pct_A = [samptime shoulder_RL_gait_Z_pct_mstd shoulder_RL_gait_Z_pct];

hip_LR_gait_Y_pct_A = [samptime hip_LR_gait_Y_pct_mstd hip_LR_gait_Y_pct];
hip_LR_gait_Z_pct_A = [samptime hip_LR_gait_Z_pct_mstd hip_LR_gait_Z_pct];

hip_RL_gait_Y_pct_A = [samptime hip_RL_gait_Y_pct_mstd hip_RL_gait_Y_pct];
hip_RL_gait_Z_pct_A = [samptime hip_RL_gait_Z_pct_mstd hip_RL_gait_Z_pct];

knee_LR_gait_Y_pct_A = [samptime knee_LR_gait_Y_pct_mstd knee_LR_gait_Y_pct];
knee_LR_gait_Z_pct_A = [samptime knee_LR_gait_Z_pct_mstd knee_LR_gait_Z_pct];

knee_RL_gait_Y_pct_A = [samptime knee_RL_gait_Y_pct_mstd knee_RL_gait_Y_pct];
knee_RL_gait_Z_pct_A = [samptime knee_RL_gait_Z_pct_mstd knee_RL_gait_Z_pct];

ankle_LR_gait_Y_pct_A = [samptime ankle_LR_gait_Y_pct_mstd ankle_LR_gait_Y_pct];
ankle_LR_gait_Z_pct_A = [samptime ankle_LR_gait_Z_pct_mstd ankle_LR_gait_Z_pct];

ankle_RL_gait_Y_pct_A = [samptime ankle_RL_gait_Y_pct_mstd ankle_RL_gait_Y_pct];
ankle_RL_gait_Z_pct_A = [samptime ankle_RL_gait_Z_pct_mstd ankle_RL_gait_Z_pct];

heel_LR_gait_Y_pct_A = [samptime heel_LR_gait_Y_pct_mstd heel_LR_gait_Y_pct];
heel_LR_gait_Z_pct_A = [samptime heel_LR_gait_Z_pct_mstd heel_LR_gait_Z_pct];

heel_RL_gait_Y_pct_A = [samptime heel_RL_gait_Y_pct_mstd heel_RL_gait_Y_pct];
heel_RL_gait_Z_pct_A = [samptime heel_RL_gait_Z_pct_mstd heel_RL_gait_Z_pct];

hip_L_angle_Y_pct_A = [samptime hip_L_angle_Y_pct_mstd(:,1:3) hip_L_angle_Y_pct];
hip_R_angle_Y_pct_A = [samptime hip_R_angle_Y_pct_mstd(:,1:3) hip_R_angle_Y_pct];

knee_L_angle_X_pct_A = [samptime knee_L_angle_X_pct_mstd(:,1:3) knee_L_angle_X_pct];
knee_R_angle_X_pct_A = [samptime knee_R_angle_X_pct_mstd(:,1:3) knee_R_angle_X_pct];

knee_L_angle_Y_pct_A = [samptime knee_L_angle_Y_pct_mstd(:,1:3) knee_L_angle_Y_pct];
knee_R_angle_Y_pct_A = [samptime knee_R_angle_Y_pct_mstd(:,1:3) knee_R_angle_Y_pct];

ankle_L_angle_Y_pct_A = [samptime ankle_L_angle_Y_pct_mstd(:,1:3) ankle_L_angle_Y_pct];
ankle_R_angle_Y_pct_A = [samptime ankle_R_angle_Y_pct_mstd(:,1:3) ankle_R_angle_Y_pct];

pelvic_angle_X_pct_A = [samptime pelvic_angle_X_pct_mstd(:,1:3) pelvic_angle_X_pct];
shoulder_angle_X_pct_A = [samptime shoulder_angle_X_pct_mstd(:,1:3) shoulder_angle_X_pct];

pelvic_angle_Z_pct_A = [samptime pelvic_angle_Z_pct_mstd(:,1:3) pelvic_angle_Z_pct];
shoulder_angle_Z_pct_A = [samptime shoulder_angle_Z_pct_mstd(:,1:3) shoulder_angle_Z_pct];

shoulder_R_angle_Y_pct_A = [samptime shoulder_R_angle_Y_pct_mstd(:,1:3) shoulder_R_angle_Y_pct];
shoulder_L_angle_Y_pct_A = [samptime shoulder_L_angle_Y_pct_mstd(:,1:3) shoulder_L_angle_Y_pct];

Trunk_R_angle_Y_pct_A = [samptime Trunk_R_angle_Y_pct_mstd(:,1:3) Trunk_R_angle_Y_pct];
Trunk_L_angle_Y_pct_A = [samptime Trunk_L_angle_Y_pct_mstd(:,1:3) Trunk_L_angle_Y_pct];





%% 5 Ploting

if Video_view=='B'

    result_output ...
        =[round(COG_Lateral_L_pct_mstd(1,2),1), round(COG_Lateral_R_pct_mstd(1,2),1), ...
        round(COG_Lateral_L_pct_mstd(1,1),0), round(COG_Lateral_R_pct_mstd(1,1),0), ...
        GAIT_cadence_A_cpm, 99, 99, ...
        GAIT_cadence_A_cpm_std, 99, 99];


    % Back segment
    writecell(Right_header,strcat(outcome_file_name,'_Back_shoulder_angle_X.csv'));
    writematrix(shoulder_angle_X_pct_A,strcat(outcome_file_name,'_Back_shoulder_angle_X.csv'),'WriteMode','append');

    writecell(Right_header,strcat(outcome_file_name,'_Back_pelvic_angle_X.csv'));
    writematrix(pelvic_angle_X_pct_A,strcat(outcome_file_name,'_Back_pelvic_angle_X.csv'),'WriteMode','append');

    Back_graph1=[round(shoulder_angle_X_pct_A(1:2:end,1:2),3),round(pelvic_angle_X_pct_A(1:2:end,2),3)];
    Back_graph2=[round(shoulder_angle_X_pct_A(:,1:2),3),round(pelvic_angle_X_pct_A(:,2),3)];

    Back_header2={'pct','shoulder angle X', 'pelvic angle X'};
    writecell(Back_header2,strcat(outcome_file_name,'_Back_graph.csv'));
    writematrix(Back_graph1,strcat(outcome_file_name,'_Back_graph.csv'),'WriteMode','append');
    writecell(Back_header2,strcat(outcome_file_name,'_Back_graph_raw.csv'));
    writematrix(Back_graph2,strcat(outcome_file_name,'_Back_graph_raw.csv'),'WriteMode','append');

elseif Video_view=='L'
    result_output ...
        =[99, 99, ...
        99, 99, ...
        GAIT_cadence_A_cpm, round(Trunk_L_angle_Y_pct_mstd(1,1),1), round(Trunk_L_angle_Y_pct_mstd(31,1),1), ...
        GAIT_cadence_A_cpm_std, round(Trunk_L_angle_Y_pct_mstd(1,4),1), round(Trunk_L_angle_Y_pct_mstd(31,4),1)];

    writecell(Left_header,strcat(outcome_file_name,'_Lateral_shoulder_joint_L.csv'));
    writematrix(shoulder_L_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_shoulder_joint_L.csv'),'WriteMode','append');

    writecell(Left_header,strcat(outcome_file_name,'_Lateral_hip_joint_L.csv'));
    writematrix(hip_L_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_hip_joint_L.csv'),'WriteMode','append');

    writecell(Left_header,strcat(outcome_file_name,'_Lateral_knee_joint_L.csv'));
    writematrix(knee_L_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_knee_joint_L.csv'),'WriteMode','append');

    writecell(Left_header,strcat(outcome_file_name,'_Lateral_ankle_joint_L.csv'));
    writematrix(ankle_L_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_ankle_joint_L.csv'),'WriteMode','append');

    Lateral_graph_L1=[round(shoulder_L_angle_Y_pct_A(1:2:end,1:2),3),round(hip_L_angle_Y_pct_A(1:2:end,2),3),round(knee_L_angle_Y_pct_A(1:2:end,2),3),round(ankle_L_angle_Y_pct_A(1:2:end,2),3)];
    Lateral_graph_L2=[round(shoulder_L_angle_Y_pct_A(:,1:2),3),round(hip_L_angle_Y_pct_A(:,2),3),round(knee_L_angle_Y_pct_A(:,2),3),round(ankle_L_angle_Y_pct_A(:,2),3)];
    Left_header2={'pct','shoulderL angle Y', 'hipL angle Y', 'kneeL angle Y', 'ankleL angle Y'};
    writecell(Left_header2,strcat(outcome_file_name,'_Lateral_graph_L.csv'));
    writematrix(Lateral_graph_L1,strcat(outcome_file_name,'_Lateral_graph_L.csv'),'WriteMode','append');
    writecell(Left_header2,strcat(outcome_file_name,'_Lateral_graph_L_raw.csv'));
    writematrix(Lateral_graph_L2,strcat(outcome_file_name,'_Lateral_graph_L_raw.csv'),'WriteMode','append');
elseif Video_view=='R'
    result_output ...
        =[99, 99, ...
        99, 99, ...
        GAIT_cadence_A_cpm, round(Trunk_R_angle_Y_pct_mstd(1,1),1), round(Trunk_R_angle_Y_pct_mstd(31,1),1), ...
        GAIT_cadence_A_cpm_std, round(Trunk_R_angle_Y_pct_mstd(1,4),1), round(Trunk_R_angle_Y_pct_mstd(31,4),1)]

    writecell(Right_header,strcat(outcome_file_name,'_Lateral_shoulder_joint_R.csv'));
    writematrix(shoulder_R_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_shoulder_joint_R.csv'),'WriteMode','append');

    writecell(Right_header,strcat(outcome_file_name,'_Lateral_hip_joint_R.csv'));
    writematrix(hip_R_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_hip_joint_R.csv'),'WriteMode','append');

    writecell(Right_header,strcat(outcome_file_name,'_Lateral_knee_joint_R.csv'));
    writematrix(knee_R_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_knee_joint_R.csv'),'WriteMode','append');

    writecell(Right_header,strcat(outcome_file_name,'_Lateral_ankle_joint_R.csv'));
    writematrix(ankle_R_angle_Y_pct_A,strcat(outcome_file_name,'_Lateral_ankle_joint_R.csv'),'WriteMode','append');

    Lateral_graph_R1=[round(shoulder_R_angle_Y_pct_A(1:2:end,1:2),3),round(hip_R_angle_Y_pct_A(1:2:end,2),3),round(knee_R_angle_Y_pct_A(1:2:end,2),3),round(ankle_R_angle_Y_pct_A(1:2:end,2),3)];
    Lateral_graph_R2=[round(shoulder_R_angle_Y_pct_A(:,1:2),3),round(hip_R_angle_Y_pct_A(:,2),3),round(knee_R_angle_Y_pct_A(:,2),3),round(ankle_R_angle_Y_pct_A(:,2),3)];
    Right_header2={'pct','shoulderR angle Y', 'hipR angle Y', 'kneeR angle Y', 'ankleR angle Y'};
    writecell(Right_header2,strcat(outcome_file_name,'_Lateral_graph_R.csv'));
    writematrix(Lateral_graph_R1,strcat(outcome_file_name,'_Lateral_graph_R.csv'),'WriteMode','append');
    writecell(Right_header2,strcat(outcome_file_name,'_Lateral_graph_R_raw.csv'));
    writematrix(Lateral_graph_R2,strcat(outcome_file_name,'_Lateral_graph_R_raw.csv'),'WriteMode','append');
end

result_output_header ...
    ={'COG L std','COG R std','COG L','COG R','Canedce','COG HS', 'COG TO', 'Cadence std', 'COG HS std', 'COG TO std'};

writecell(result_output_header,strcat(outcome_file_name,'_summary.csv'));
writematrix(result_output,strcat(outcome_file_name,'_summary.csv'),'WriteMode','append');

