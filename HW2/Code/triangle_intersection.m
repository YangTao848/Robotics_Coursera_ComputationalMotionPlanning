function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

% TODO: Didn't implement if one triangle is inside another one

flag = false;

for i = 1:3
    ii = i+1;
    if (ii > 3)
        ii = 1;
    end
    
    for j = 1:3
        jj = j + 1;
        if (jj > 3)
           jj = 1; 
        end
       flag = line_intersection(P1(i,:),P1(ii,:),P2(j,:),P2(jj,:)); 
       if (flag == true)
           return
       end
    end
end

% *******************************************************************

end
    
% This function was implemented based on the theories found below:
% http://math.stackexchange.com/questions/274712/calculate-on-which-side-of-straign-line-is-dot-located
% http://stackoverflow.com/questions/2778240/detection-of-triangle-collision-in-2d-space/2778252