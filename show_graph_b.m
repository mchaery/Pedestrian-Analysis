function[] = show_graph_b(Case_name1,Case_name2)

xlabel_list =["shoulder","pelvic"];
title_list =["어깨 수평 각도","골반 수평 각도"];
set(figure,'position',[100 100 1500 800])%
yourpath = pwd;
for i = 1:2
    %% case 1
    userpath(yourpath)
    subplot(3,2,i)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i-1);
    general_y2 = reference_data(:,2*i);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    newpath1=append(yourpath,'\',Case_name1,'\output');
    userpath(newpath1)

    data1=append(Case_name1,'_result_Back_',xlabel_list(i),'_angle_X.csv');
    data1=load(data1);

    x = data1(:,1)*100;
    y1 = data1(:,5:end); %개별 걸음
    subplot(3,2,i)
    plot(x,y1,'Color', '#aaaaaa', 'LineWidth',1);
    hold on
    y2 = data1(:,3:4); %mean+-sd
    subplot(3,2,i)
    plot(x,y2,'Color','#63CC63', 'LineWidth',2);
    hold on
    y3 = data1(:,2); %mean
    subplot(3,2,i)
    p = plot(x,y3,'Color','#006400', 'LineWidth',3);
    xlabel('보행(%)')
    ylabel('우측하강-우측기상(deg)')
    legend(p,{'Case1'});
    title(title_list(i))
    hold off

    %% case 2
    userpath(yourpath)
    subplot(3,2,i+2)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i-1);
    general_y2 = reference_data(:,2*i);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    newpath2=append(yourpath,'\',Case_name2,'\output');
    userpath(newpath2)

    data2=append(Case_name2,'_result_Back_',xlabel_list(i),'_angle_X.csv');
    data2=load(data2);

    x = data2(:,1)*100;
    y1 = data2(:,5:end); %개별 걸음
    subplot(3,2,i+2)
    plot(x,y1,'Color', '#aaaaaa', 'LineWidth',1);
    hold on
    y2 = data2(:,3:4); %mean+-sd
    subplot(3,2,i+2)
    plot(x,y2,'Color','#D873F1', 'LineWidth',2);
    hold on
    y3 = data2(:,2); %mean
    subplot(3,2,i+2)
    p = plot(x,y3,'Color','#9400D3', 'LineWidth',3);
    xlabel('보행(%)')
    ylabel('우측하강-우측기상(deg)')
    legend(p,{'Case2'});
    title(title_list(i))
    hold off

    %% case 1,2 비교
    userpath(yourpath)
    subplot(3,2,i+4)
    reference_data=importdata('reference_CES48.csv');%일반인 데이터(회색 영역)
    general_x=0:2:100;
    general_y1 = reference_data(:,2*i-1);
    general_y2 = reference_data(:,2*i);
    shade(general_x, general_y1,general_x, general_y2,'FillType',[1 2;2 1]);
    hold on

    x = data2(:,1)*100;
    y1 = data1(:,3:4); %case1 mean+-sd
    subplot(3,2,i+4)
    p1 = plot(x,y1,'Color','#63CC63', 'LineWidth',2);
    hold on
    y2 = data1(:,2);  data1(:,2), data2(:,3), data2(:,4), %case 1 mean
    subplot(3,2,i+4)
    p2 = plot(x,y2,'Color','#006400', 'LineWidth',3);
    y3 = data2(:,3:4); %case 2 mean+-sd
    subplot(3,2,i+4)
    p3 = plot(x,y3,'Color','#D873F1', 'LineWidth',2);
    hold on
    y4 = data2(:,2); %case 2 mean
    subplot(3,2,i+4)
    p4 = plot(x,y4,'Color','#9400D3', 'LineWidth',3);

    xlabel('보행(%)');
    ylabel('우측하강-우측기상(deg)');
    legend([p2 p4],{'Case1','Case2'});
    title(title_list(i));
    hold off

    resultpath_png = append(yourpath,'\result_Back.png');
    saveas(gcf,resultpath_png)
end
%% 비교데이터를 .csv 저장
Result_output = horzcat(data1(:,2), data1(:,3), data1(:,4), data2(:,2),data2(:,3), data2(:,4));
Result_output_header = ["1_mean", "1_mean-sd", "1_mean+sd", "2_mean", "2_mean-sd", "2_mean+sd"];
resultpath_csv = append(yourpath,'\result_Back.csv');
writematrix(Result_output_header, resultpath_csv);
writematrix(Result_output,resultpath_csv,'WriteMode','append');

