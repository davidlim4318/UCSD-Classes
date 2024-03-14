clear

ks = zeros(5,1);

for i = 1:5
    M = readmatrix("exp1trial" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = M(:,5);
    x2 = M(:,6);
    F = M(:,8);
    
    figure(1)
    clf
    hold on
    
    yyaxis left
    plot(time,x1,'b.','MarkerSize',10)
    plot(time,x2,'r.','MarkerSize',10)
    ylabel("Displacement (counts)")
    ax = gca;
    ax.YColor = 'k';
    
    yyaxis right
    plot(time,F,'m.','MarkerSize',10)
    ylabel("Force (V)")
    ax = gca;
    ax.YColor = 'm';
    
    hold off
    title("(a)")
    xlabel("Time (s)")
    legend('Disc 1','Disc 2','Control Effort','Location','best')
    
    ax = gca;
    ax.TitleHorizontalAlignment = 'left';
    set(ax,'FontSize',12)

    idx = find(abs(diff(F)) > 0,1,'last');
    ks(i) = abs(0.5./(x1(idx) - x2(idx)));

    pause
end

k = mean(ks);
