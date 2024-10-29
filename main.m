%% 케이스선택 
clc, clear
Video_view = input('case 방향을 입력하세요(R/B) : ','s');
if (Video_view=='r')||(Video_view=='R')
    Video_view = 'R';
elseif (Video_view == 'b')||(Video_view=='B')
    Video_view = 'B';
else
    disp('방향을 다시 입력하세요')
    pause(3)
    return
end

s1 = input('첫번째 케이스의 번호를 입력하세요(1/2/3): ','s');
s2 = input('두번째 케이스의 번호를 입력하세요(1/2/3): ','s');

if (Video_view == 'R')
    s = 'Right_case_';
else 
    s = 'Back_case_';
end

Case_name1 = append(s,s1);
Case_name2 = append(s,s2);
%% 계산

calculation(Case_name1, Video_view)
calculation(Case_name2, Video_view)

%% 그래프
if (Video_view == 'R')
    show_graph_r(Case_name1,Case_name2)
else
    show_graph_b(Case_name1,Case_name2)
end
