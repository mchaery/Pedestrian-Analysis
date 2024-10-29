%% select case
clc, clear
Video_view = input('enter the case direction(R/B) : ','s');
if (Video_view=='r')||(Video_view=='R')
    Video_view = 'R';
elseif (Video_view == 'b')||(Video_view=='B')
    Video_view = 'B';
else
    disp('Please re-enter the direction.')
    pause(3)
    return
end

s1 = input('Enter the number for the first case (1/2/3):','s');
s2 = input('Enter the number for the second case (1/2/3):','s');

if (Video_view == 'R')
    s = 'Right_case_';
else 
    s = 'Back_case_';
end

Case_name1 = append(s,s1);
Case_name2 = append(s,s2);
%% calculation

calculation(Case_name1, Video_view)
calculation(Case_name2, Video_view)

%% visualize, save data/graph
if (Video_view == 'R')
    show_graph_r(Case_name1,Case_name2)
else
    show_graph_b(Case_name1,Case_name2)
end
