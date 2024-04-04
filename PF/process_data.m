function process_data(X,Y,T)
    data = zeros(1081,2);
    M = zeros(50,2);
    particles = zeros(10*500,2);

    % Set up graphs, figures --------------------------------------------------
    figure;hold off;plot(0,0,'.');axis auto;zoom on, grid on; hold on;
    hdl = get(gcf, 'userdata');
    hdl.PlotRobot = plot(data(:,1),data(:,2),'color','b','marker','.','lineStyle','none');
    hdl.PlotLines = plot(M(:,1),M(:,2),'color','r','marker','*','lineStyle','none');
    hdl.PlotParticles = plot(particles(:,1),particles(:,2),'color','m','marker','.','lineStyle','none','MarkerSize',2);%,'MarkerEdgeColor','k','MarkerFaceColor','b');
    % axis equal;zoom on, grid on;hold on;
    axis ([-5 5 -5 5]); grid on;

    init=0;

    for ii=2:size(X,2)
        %calculate x,y
        x=X(ii);
        y=Y(ii);

        if (init ==0)
            [pf] = particleFilter;
            init =1;            
        else
            deltaT = T(ii)-T(ii-1);
            data=[x,y];
            robotPose.pose=[0;0;0];
            [pf] = particleFilter(pf,data,robotPose,deltaT,T,ii);
            draw_particles(hdl,pf,particles) ;           
        end

%         data=[];
%         data(:,1) = ranges(ii,:).* cos(theta);
%         data(:,2) = ranges(ii,:).* sin(theta);
%         set(hdl.PlotRobot,'Xdata',data(:,1), 'Ydata', data(:,2));    
        set(hdl.PlotLines,'Xdata',x, 'Ydata', y); 	

        fprintf(1,'%d\n',ii);
        drawnow;
    end
end

function draw_particles(hdl,pf,particles)

%     for i=1:size(pf.filter,2)
%         particles(i,:)=[pf.filter(i).hypothesis.pose(1),pf.filter(i).hypothesis.pose(3)];
%     end
    count=1;
    for i=1:size(pf.filter,2)
        for j=1:size(pf.filter(i).sampleSet,2)
            particles(count,:)=[pf.filter(i).sampleSet(j).pose(1),pf.filter(i).sampleSet(j).pose(3)];
            count=count+1;
        end
    end    
    
  	set(hdl.PlotParticles,'Xdata',particles(:,1), 'Ydata', particles(:,2)); 	
    
end



