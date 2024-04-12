function process_data(X,Y,T, anchor_ID, anchorlocations)
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

    R = eul2rotm([anchorlocations.psi, anchorlocations.theta, anchorlocations.phi], 'ZYX');


    anchor_order = [1,5,2,7,4];
    % color_order = ["#EDB120", "#0072BD", "#7E2F8E", "#A2142F", "#77AC30"];
    color_order = ['r', 'g', 'b', 'm', 'y'];

    for i = 1:length(anchorlocations.x)
    % Extract the rotation matrix for the current point
    R_current = R(:,:,i);
    
    % Define a fixed vector representing the initial direction (e.g., facing east)
    v = [1; 0; 0];
    
    % Transform the fixed vector to the local coordinate system
    local_v = R_current * v;
    
    % Extract the local x and y components of the transformed vector
    local_x = local_v(1);
    local_y = local_v(2);
    
    % Choose the color for the current triangle
    color_index = mod(i, length(color_order)) + 1; % Cycling through the color order
    color = color_order(color_index);
    
    ang = atan2(local_y,local_x);
    ang = ang+pi;

    fprintf(1,'%d: Angle %4.2f\n',i,ang+pi)
    txt = sprintf("%d",anchor_order(i));
    Veh = vehicle([anchorlocations.x(i), anchorlocations.y(i), ang]);
    patch(Veh(1:3), Veh(4:6),  color,'FaceAlpha', 0.5);
    text(anchorlocations.x(i)+0.4, anchorlocations.y(i),txt);

end

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



