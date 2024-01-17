function [us1,us2,us3,us4,us5] = simuAffichage(us_vector,us1,us2,us3,us4,us5)
%simuAffichage helps to visualize graphically the processes going on in the
%class simulation (that simulates the work of u.s. sensors)
%% !roundrobin graphics

switch us_vector(1)
    case 1
        set(us1,'Visible', 'off');
    case 0
        set(us1,'Visible', 'off');
end
switch us_vector(2)
    case 1
        set(us2,'Visible', 'off');
    case 0
        set(us2,'Visible', 'off');
end
switch us_vector(3)
    case 1
        set(us3,'Visible', 'off');
    case 0
        set(us3,'Visible', 'off');
end
switch us_vector(4)
    case 1
        set(us4,'Visible', 'off');
    case 0
        set(us4,'Visible', 'off');
end
switch us_vector(5)
    case 1
        set(us5,'Visible', 'off');
    case 0
        set(us5,'Visible', 'off');
end

drawnow

% % x2 = [2 ; 2 ; 8 ];
% % y2 = [4 ; 8 ; 4 ];
% % aaa = patch(x2,y2,'green')
% % x2 = [2 ; 2 ; 8 ];
% % y2 = [4 ; 8 ; 4 ];
% % bbb = patch(x2,y2,'blue')
% % set(bbb,'Visible', 'off');
end

