function[] = show_graph_r(Case_name1,Case_name2)

set(figure,'position',[100 100 1500 800])%
xlabel_list =["shoulder","hip","knee","ankle"];
ylabel_list =["신전-굴곡(deg)","신전-굴곡(deg)","저측굴곡-배측굴곡(deg)","신전-굴곡(deg)"];
title_list =["우측 어깨 각도","우측 골반 각도","우측 무릎 각도","우측 발목 각도"];
yourpath = pwd;
for i = 1:4
    %% case 1
    userpath(yourpath)
    subplot(3,4,i)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i+3);
    general_y2 = reference_data(:,2*i+4);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    newpath1=append(yourpath,'\',Case_name1,'\output');
    userpath(newpath1)

    data1=append(Case_name1,'_result_Lateral_',xlabel_list(i),'_joint_R.csv');
    data1=load(data1);

    x = data1(:,1)*100;
    y1 = data1(:,5:end); %개별 걸음
    subplot(3,4,i)
    plot(x,y1,'Color', '#aaaaaa', 'LineWidth',1);
    hold on
    y2 = data1(:,3:4); %mean+-sd
    subplot(3,4,i)
    plot(x,y2,'Color','#90AFFF', 'LineWidth',2);
    hold on
    y3 = data1(:,2); %mean
    subplot(3,4,i)
    p = plot(x,y3,'Color','#0000FF', 'LineWidth',3);

    xlabel('보행(%)')
    ylabel(ylabel_list(i))
    legend(p,{'Case1'});
    title(title_list(i))
    hold off

    %% case 2
    userpath(yourpath)
    subplot(3,4,i+4)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i+3);
    general_y2 = reference_data(:,2*i+4);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    newpath2=append(yourpath,'\',Case_name2,'\output');
    userpath(newpath2)

    data2=append(Case_name2,'_result_Lateral_',xlabel_list(i),'_joint_R.csv');
    data2=load(data2);

    x = data2(:,1)*100;
    y1 = data2(:,5:end); %개별 걸음
    subplot(3,4,i+4)
    plot(x,y1,'Color', '#aaaaaa', 'LineWidth',1);
    hold on
    y2 = data2(:,3:4); %mean+-sd
    subplot(3,4,i+4)
    plot(x,y2,'Color','#FF87D2', 'LineWidth',2);
    hold on
    y3 = data2(:,2); %mean
    subplot(3,4,i+4)
    p = plot(x,y3,'Color','#FF1493', 'LineWidth',3);

    xlabel('보행(%)')
    ylabel(ylabel_list(i))
    legend(p,{'Case2'});
    title(title_list(i))
    hold off

    %% case 1,2 비교
    userpath(yourpath)
    subplot(3,4,i+8)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i+3);
    general_y2 = reference_data(:,2*i+4);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    x = data1(:,1)*100;
    y1 = data1(:,3:4); %case1 mean+-sd
    subplot(3,4,i+8)
    p1 = plot(x,y1,'Color','#90AFFF', 'LineWidth',2);
    hold on
    y2 = data1(:,2); %case1 mean
    subplot(3,4,i+8)
    p2 = plot(x,y2,'Color','#0000FF', 'LineWidth',3);
    hold on
    y3 = data2(:,3:4); %case2 mean+-sd
    subplot(3,4,i+8)
    p3 = plot(x,y3,'Color','#FF87D2', 'LineWidth',2);
    hold on
    y4 = data2(:,2); %case1 mean
    subplot(3,4,i+8)
    p4 = plot(x,y4,'Color','#FF1493', 'LineWidth',3);

    xlabel('보행(%)');
    ylabel(ylabel_list(i));
    legend([p2 p4],{'Case1','Case2'});
    title(title_list(i));
    hold off

    resultpath_png = append(yourpath,'\result_Right.png');
    saveas(gcf,resultpath_png)
end
%% 비교데이터를 .csv 저장
Result_output = horzcat(data1(:,2), data1(:,3), data1(:,4), data2(:,2),data2(:,3), data2(:,4));
Result_output_header = ["1_mean", "1_mean-sd", "1_mean+sd", "2_mean", "2_mean-sd", "2_mean+sd"];
resultpath_csv = append(yourpath,'\result_Right.csv');
writematrix(Result_output_header, resultpath_csv);
writematrix(Result_output,resultpath_csv,'WriteMode','append');

