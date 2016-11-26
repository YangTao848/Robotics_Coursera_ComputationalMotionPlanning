function flag = line_intersection(P1, P2, P3, P4)

% Line intersection test : returns true if the line segments intersects 
% each other and false otherwise

% The code was implemented based on the theory explained in the link below:
% https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/#line_line_intersection


flag = false;

A1 = P2(2) - P1(2);
B1 = P1(1) - P2(1);
C1 = A1 * P1(1) + B1 * P1(2);

A2 = P4(2) - P3(2);
B2 = P3(1) - P4(1);
C2 = A2 * P3(1) + B2 * P3(2);

det = (A1 * B2) - (A2 * B1);

if (det == 0)       % lines are parallel
    return
else
    x = (B2*C1 - B1*C2)/det;
    y = (A1*C2 - A2*C1)/det;

    % These four lines are to get rid of zeros after decimal point.
    % Becasue 3.0000 is not equalt to 3 in MATLAB
    temp = int32(x * 10000);
    x = double(temp)/10000;
    temp= int32(y * 10000);
    y = double(temp)/10000;
    
    
    
    if(P1(1) >= P2(1))
        minX12 = P2(1);
        maxX12 = P1(1);
    else
        minX12 = P1(1);
        maxX12 = P2(1);
    end
    if(P3(1) >= P4(1))
        minX34 = P4(1);
        maxX34 = P3(1);
    else
        minX34 = P3(1);
        maxX34 = P4(1);
    end
    
    if( (x >= minX12) && (x <= maxX12) && (x >= minX34) && (x <= maxX34) )
        if( (y >= min(P1(2),P2(2))) && y <= max(P1(2),P2(2)) && (y >= min(P3(2),P4(2))) && y <= max(P3(2),P4(2)) )
            flag = true;
        end
    end
    
    
%     if( (x >= min(P1(1),P2(1))) && x <= max(P1(1),P2(1)) && (x >= min(P3(1),P4(1))) && x <= max(P3(1),P4(1)) )
%         if( (y >= min(P1(2),P2(2))) && y <= max(P1(2),P2(2)) && (y >= min(P3(2),P4(2))) && y <= max(P3(2),P4(2)) )
%             flag = true;
%         end
%     end
end

end