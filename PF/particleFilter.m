% Need some notation on structures
%     struct Feature
%         double x,y;
%         int numberOfBeams; //number of beams which contributed to this feature
%         bool associated;
% 
%     struct Sample
%         double pose(4);
%         int mode; //in which mode of motion is this sample, initialised as -1
%         double w; //weight
% 
%     struct Hypothesis
%         double pose(4); //x,Vx,y,Vy
%         double cov(4); //only diagonal elements
% 
%     struct Filter
%       std::vector<Sample> sampleSet;
%       Hypothesis hypothesis;
%       std::vector<Hypothesis> hypotheses; //the sequence of hypotheses; last in array is current
%       int targetID;
%       int featureIndex; //index of the associated feature, initialised as -1
%       int validityCounter;  //after three succesfull observations ->set validTrack=true
%                             //after three times without association -> delete track
%       bool validTrack; //starts as false
%       bool trackInitiated;
% 
%     struct humanMotion
%         double weight; //probability of this motion
%         double speed, turnrate;
% 
%     struct RobotPose
%         double pose(3);
%         double cov(3);
% 
%     struct SensorIntrinsics
%         double offset(5); //x,y,r,p,y
%         double rangeVariance;
%
%    so the total struct
%   pf.filter
%       pf.filter().sampleSet().mode
%       pf.filter().targetID
%   pf.humanMotion
%   pf.robotPose

function [pf] = particleFilter(pf,data,robotPose,deltaT,T,ii)
 if (nargin < 1)
     [pf] = initParticleFilter;
 else
     [pf] = trackingLoop(pf,data, robotPose,deltaT,T,ii);
 end
end

% one iteration of the tracking loop   
function [pf]= trackingLoop(pf,data, robotPose,deltaT,T,ii)

    features=[];

    for i=1:size(data,1)
       f.x = data(i,1);
       f.y = data(i,2);
 
       f.associated = false;
	   fprintf(1,'arrived observation %d :[%6.4f,%6.4f] dt:%6.4f\n', i , f.x , f.y ,deltaT);

        R = sqrt((f.x*f.x)+(f.y*f.y));
        theta = atan2(f.y,f.x);

%       %adjust the feature by angle;
%       theta += upperBodyData.angles.pan;
       f.x = R * cos(theta);
       f.y = R * sin(theta);
%        fprintf(1,'Detected person [x,y] = [%6.4f,%6.4f]',f.x, f.y );
% 
       features=[features,f];
    end
    fprintf(1,'Size of features :%d\n',size(features,2));

    [features ] = transformToGlobal(features,robotPose);
    %associate features with predictions
    [pf,features] = associateData(pf,features);
    
%     std::cout<<"N       tracks: "<<this->filterSet_.size()<<std::endl;
%     int c=0;
%     for(unsigned int i=0; i<this->filterSet_.size();i++)
%         if(this->filterSet_[i].validTrack==true)
%             c++;
%     end
%     std::cout<<"N valid tracks: "<<c<<std::endl;
% 
%     for(unsigned int i=0; i<this->filterSet_.size();i++)
%         if(this->filterSet_[i].validTrack==false)
%             std::cout<<i<<"   invalid   "<< this->filterSet_[i].validityCounter<<std::endl;
%         else
%             std::cout<<i<<"   valid   "<< this->filterSet_[i].validityCounter<<std::endl;
%         end
%    end
% 

    %predict motion
     [pf] = prediction (pf, deltaT);

    %initiate new tracks for unassociated features (equals a new filter for a new target)
    [pf] = initiateTracks(pf,features);
    
    %remove tracks for lost features
    [pf] = deleteTracks(pf);

    %weigh samples
    [pf] = personRangeModel(pf,features);

    %get statistics for the prediction so we can do data association
    [pf] = calculateStatisticsPF(pf,T,ii);

    %resample all filters
    [pf] = resample(pf);
end


function [pf] = initParticleFilter

    %initialise a motion model
    %constant velocity, constant turnrate
    h.weight=0.9;
    h.turnrate=0;
    humanModel(1)=h;
    h.weight=0.05;
    h.turnrate=20*pi/180;
    humanModel(2)=h;
    h.weight=0.05;
    h.turnrate=-20*pi/180;
    humanModel(3)=h;
    
    pf.humanModel=humanModel;

%     modeChangeProb=0.025;
%     std::vector<double> row;
%     for(unsigned int i=0;i<humanModel_.size();i++)
%     {
%         modeChangeMatrix_.push_back(row);
%         for(unsigned int j=0;j<humanModel_.size();j++)
%         {
%             if(i==j)
%                 modeChangeMatrix_(i).push_back(1.-2.*modeChangeProb);
%             else
%                 modeChangeMatrix_(i).push_back(modeChangeProb);
%         }
%     }
% 
%     for(unsigned int i=0;i<humanModel_.size();i++)
%     {
%         for(unsigned int j=0;j<humanModel_.size();j++)
%         {
%             fprintf(1,'modeChangeMatrix_(i)(j)<<"  ";
%         }
%         fprintf(1,'std::endl;
%     }
%     fprintf(1,'std::endl;

    %init random number generator
%     rng_ = gsl_rng_alloc (gsl_rng_taus);
%     gsl_rng_set (rng_, time(0));
    %RandStream.setDefaultStream (RandStream('mt19937ar','seed',sum(100*clock)));

    pf.lastTargetID_=1;

    for i=1:3
        robotPose.cov(i)=0.;
        robotPose.pose(i)=0.;
    end
    fprintf(1,'Initialising robot pose %6.4f,%6.4f,%6.4f\n',robotPose.pose(1) ,robotPose.pose(2),robotPose.pose(3));

    pf.associationGate = 10;
    pf.trackValidationCount =  8;
    pf.trackDeletionCount =  -3;
    pf.MAXSAMPLES = 500;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% UPDATE VARIABLE CONSTANTS HERE: 

    pf.personRangeModel_variance = 0.75;
    
    pf.prediction_velocity_x = 0.05;
    pf.prediction_velocity_xDot = 0.05;
    pf.prediction_velocity_y = 0.05;
    pf.prediction_velocity_yDot = 0.05;
    
    pf.resample_x = 0.0050;
    pf.resample_xDot = 0.0050;
    pf.resample_y = 0.0050;
    pf.resample_yDot = 0.0050;

    pf.outpute_Filename = 'PF_var_0.75.csv';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    fprintf(1,'Init params, association gate:%6.4f  track validation count:%6.4f  track deletion count:%6.4f\n',  pf.associationGate, pf.trackValidationCount, pf.trackDeletionCount);
end
 
%alen - currently driven by noise and constant velocity model? velocity also driven by noise?
function [pf] = prediction (pf, deltaT)
    
    if( (myIsField (pf, 'filter') ) ==0)
            return;
    end

    % i=0:;i<filterSet_.size();i++)
    for i=1:size(pf.filter,2)
    %         for(unsigned int j=0;j<filterSet_(i).sampleSet.size();j++)
        for j=1:size(pf.filter(i).sampleSet,2)
    %          filterSet_(i).sampleSet(j).pose(0) += filterSet_(i).sampleSet(j).pose(1)+gsl_ran_gaussian(rng_, 0.1))*deltaT;
            pf.filter(i).sampleSet(j).pose(1) = pf.filter(i).sampleSet(j).pose(1)  +  ( (pf.filter(i).sampleSet(j).pose(2) + gsl_ran_gaussian(pf.prediction_velocity_x)) *deltaT);         
            pf.filter(i).sampleSet(j).pose(2) = pf.filter(i).sampleSet(j).pose(2)  + gsl_ran_gaussian(pf.prediction_velocity_xDot);
            pf.filter(i).sampleSet(j).pose(3) = pf.filter(i).sampleSet(j).pose(3)  +  ( (pf.filter(i).sampleSet(j).pose(4) + gsl_ran_gaussian(pf.prediction_velocity_y)) *deltaT);          
            pf.filter(i).sampleSet(j).pose(4) = pf.filter(i).sampleSet(j).pose(4)  + gsl_ran_gaussian(pf.prediction_velocity_yDot);
        end
    end

end %end function

function [pf,features] = associateData(pf,features)

     %if there is no object tracked the feature will be used to iniate a new one
     if( (myIsField (pf, 'filter') ) ==0)
            return;
    end
    
    %first assign all observations to not be associated
    for  j=1:size(pf.filter,2)
      pf.filter(j).featureIndex=-1;
%       fprintf(1,'%d, pose %6.4f %6.4f %6.4f\n', j,pf.filter(j).hypothesis.pose(1) , pf.filter(j).hypothesis.pose(3) );
    end



    %  find closest match within gate
    %  the internal logic of this piece of code cycles through all features and finds one within the gate distance
    %  to the tracks, marking it as associated. this basically makes each feature only associted with one track.
    for i=1:size(features,2)

    %is the feature already associated with a filter?
        if(features(i).associated==true)
                continue;
        end
        minDist=999999.; %Set something ridiculous so all matches considered
        closest=-1;
        for j=1:size(pf.filter,2)
            %if filter has association already
            if(pf.filter(j).featureIndex ~= -1)
                continue;
            else
                a=(pf.filter(j).hypothesis.pose(1) - features(i).x)*(pf.filter(j).hypothesis.pose(1) - features(i).x);
                b=(pf.filter(j).hypothesis.pose(3) - features(i).y)*(pf.filter(j).hypothesis.pose(3) - features(i).y);
                temp=sqrt(a+b);
%                 fprintf(1,'filter: %d, pose [%6.4f,%6.4f] feature [%6.4f,%6.4f] dist:%6.4f\n',i,pf.filter(j).hypothesis.pose(1),pf.filter(j).hypothesis.pose(3),features(i).x,features(i).y,temp);
                 if(temp<pf.associationGate && temp<minDist)
                     %remember closest
                     minDist=temp;
                     closest=j;
                 end
            end
        end %for all filters

        %check if there is indeed an association
         if(closest ~=-1)
             %associate closest and i
             pf.filter(closest).featureIndex=i;
             if(pf.filter(closest).validityCounter>=pf.trackValidationCount)
%                  fprintf(1,'Set valid:%d\n',pf.filter(closest).targetID);%alen- debug
                 pf.filter(closest).validTrack=true;
             else
                 pf.filter(closest).validityCounter=pf.filter(closest).validityCounter+1;
             end
             features(i).associated=true;
%                 fprintf(1,'"associate: "<<i <<"  "<<closest<<"  "<<pf.filter(closest).validityCounter<<std::endl;
         end
    end%for all features

    %check for unassociated tracks
    for i=1:size(pf.filter,2)
        if(pf.filter(i).featureIndex ==-1)
             if(pf.filter(i).validityCounter<=pf.trackDeletionCount)
                 pf.filter(i).validTrack=false;
%                  fprintf(1,' Set invalid:%d\n', pf.filter(i).targetID);%alen- debug
             else
                 pf.filter(i).validityCounter= pf.filter(i).validityCounter-1;
             end
        end
    end
    
end%end function

function [pf] = initiateTracks(pf,features)

    %NOTES
    % C++
    % double gsl_ran_flat (const gsl_rng * r, double a, double b)
    % This function returns a random variate from the flat (uniform) distribution from a to b. The distribution is, 
    % MATLAB
    % Generate values from the uniform distribution on the interval (a, b).
    %  r = a + (b-a).*rand(100,1);

     %for all not associated features, initialise a new filter
    for i=1:size(features,2)
        %is the feature already associated with a filter?
        if(features(i).associated==true)
            continue;
        else

            %we are tracking people, so expect speeds accordingly
            %spawn samples around the features location and associate the two
            newFilter.featureIndex=i;
            newFilter.validityCounter=0;
            newFilter.validTrack=false;
            newFilter.targetID=pf.lastTargetID_;
            pf.lastTargetID_= pf.lastTargetID_ + 1;

            %initialise samples
            for j=1:pf.MAXSAMPLES
    %             Sample s;
    %             double polarPose(2);
    %             double sinus, cosinus;
                polarPose(1)= gsl_ran_flat(-0.2, +0.2); %radius
                polarPose(2)= gsl_ran_flat(0., pi*2.); %angle
    %             sincos(polarPose(1), &sinus, &cosinus);
                [sinus,cosinus] = sincos(polarPose(2));
                s.pose(1) = polarPose(1)*cosinus + features(i).x;   %x
                s.pose(2) = gsl_ran_flat(-2., 2.);                          %Vx
                s.pose(3) = polarPose(1)*sinus + features(i).y;       %y
                s.pose(4) = gsl_ran_flat( -2., 2.);                         %Vy
                s.mode = chooseMode(pf,gsl_ran_flat( 0., 1.));

                newFilter.sampleSet(j)=s;
            end

            newFilter.hypothesis.pose(1)=features(i).x;
            newFilter.hypothesis.pose(2)=0;
            newFilter.hypothesis.pose(3)=features(i).y;
            newFilter.hypothesis.pose(4)=0;
            
            if( (myIsField (pf, 'filter') ) ==0)
                lastFilter =0;
            else
                lastFilter = size(pf.filter,2);
            end
            pf.filter(lastFilter+1) = newFilter;
        end
    end
end %end function

function [i] = chooseMode(pf,randonNumber)
    thresh=0.;
    for i=1:size(pf.humanModel,2)

        thresh= thresh + pf.humanModel(i).weight;
        if(randonNumber<=thresh)
              return;
        end
    end
    fprintf(1,'ParticleFilter::chooseMode\n');
    i=1;
end

% function [i] = changeMode (randonNumber,mode)
%     thresh=0.;
%     for i=1:size(modeChangeMatrix_(mode),2);
% 
%         thresh = thresh + modeChangeMatrix_(mode)(i);
%         if(randonNumber<thresh)
%                  return;
%         end
%     end
%     fprintf(1,'ParticleFilter::changeMode\n');
%     i=1;
% end

function [pf] = calculateStatisticsPF(pf,T,ii)
    for i=1:size(pf.filter,2)

        pose =zeros(1,4);
        cov = zeros(1,4);
        tempW=0;
        
%         printFilterStats(pf,i,tempW);
        
        for j=1:size(pf.filter(i).sampleSet,2)

            %compute weighted mean
            pose(1) = pose(1) + (pf.filter(i).sampleSet(j).w*pf.filter(i).sampleSet(j).pose(1));
            pose(2) = pose(2) + (pf.filter(i).sampleSet(j).w*pf.filter(i).sampleSet(j).pose(2));
            pose(3) = pose(3) + (pf.filter(i).sampleSet(j).w*pf.filter(i).sampleSet(j).pose(3));
            pose(4) = pose(4) + (pf.filter(i).sampleSet(j).w*pf.filter(i).sampleSet(j).pose(4));
            tempW = tempW + (pf.filter(i).sampleSet(j).w);
        end

        pf.filter(i).hypothesis.pose(1)=pose(1);
        pf.filter(i).hypothesis.pose(2)=pose(2);
        pf.filter(i).hypothesis.pose(3)=pose(3);
        pf.filter(i).hypothesis.pose(4)=pose(4);

        for j=1:size(pf.filter(i).sampleSet,2)
            %compute weighted cov
            cov(1) = cov(1) + ((pf.filter(i).sampleSet(j).pose(1)-pf.filter(i).hypothesis.pose(1))*(pf.filter(i).sampleSet(j).pose(1)-pf.filter(i).hypothesis.pose(1)));
            cov(2) = cov(2) + ((pf.filter(i).sampleSet(j).pose(2)-pf.filter(i).hypothesis.pose(2))*(pf.filter(i).sampleSet(j).pose(2)-pf.filter(i).hypothesis.pose(2)));
            cov(3) = cov(3) + ((pf.filter(i).sampleSet(j).pose(3)-pf.filter(i).hypothesis.pose(3))*(pf.filter(i).sampleSet(j).pose(3)-pf.filter(i).hypothesis.pose(3)));
            cov(4) = cov(4) + ((pf.filter(i).sampleSet(j).pose(4)-pf.filter(i).hypothesis.pose(4))*(pf.filter(i).sampleSet(j).pose(4)-pf.filter(i).hypothesis.pose(4)));
        end

        pf.filter(i).hypothesis.cov(1) = sqrt(cov(1)/size(pf.filter(i).sampleSet,2));
        pf.filter(i).hypothesis.cov(2) = sqrt(cov(2)/size(pf.filter(i).sampleSet,2));
        pf.filter(i).hypothesis.cov(3) = sqrt(cov(3)/size(pf.filter(i).sampleSet,2));
        pf.filter(i).hypothesis.cov(4) = sqrt(cov(4)/size(pf.filter(i).sampleSet,2));

        printFilterStats(pf, i, tempW, pf.outpute_Filename,T,ii);
    end
end % end function

function [pf] = deleteTracks(pf)

    priorToDelete = size(pf.filter,2);

    i=1;
    while  i <(size(pf.filter,2)+1)
% %        %alen - just for kicks am gonna print out the covariance to check
% %        fprintf(1,' __func__  << i << "ID=" << pf.filter(i).targetID << " (" <<
% %                pf.filter(i).hypothesis.cov(0) , ...
% %                pf.filter(i).hypothesis.cov(1) , ...
% %                pf.filter(i).hypothesis.cov(2) , ...
% %                pf.filter(i).hypothesis.cov(3) << ")" << std::endl;
        if(pf.filter(i).validTrack==false && pf.filter(i).validityCounter<=-2)
            fprintf(1,'Killed:%d\n',pf.filter(i).targetID);
%             pf.filter.erase(pf.filter.begin()+i);
            pf.filter(i)=[];
        else
            i=i+1;
        end
    end
    fprintf(1,'deleted:%d\n' ,size(pf.filter,2) - priorToDelete);
end

% function [allSamples] = getAllSamples(allSamples,pf)
% 
%     for i=1:size(pf.filter,2)
%         for j=1:size(pf.filter(i).sampleSet,2)
%             allSamples=[allSamples,pf.filter(i).sampleSet(j)];
%         end
%     end
%     
% end

function [pf] = resample(pf)
    for i=1:size(pf.filter,2)
        [pf] = resample_systematic(pf,i);
    end
end

function [pf] = resample_systematic(pf,i)
    N=size(pf.filter(i).sampleSet,2);
    M = 1./size(pf.filter(i).sampleSet,2);

    resampledParticles=[];
%     resampledParticles.reserve(pf.filter(i).sampleSet.size());

%     U = gsl_ran_flat(rng_, 0, M);
    U = gsl_ran_flat(0,M);
    S = 0.;
    k = 0;
    for m=1:N
        S = S + pf.filter(i).sampleSet(m).w;
%         fprintf(1,'%d\n ',m);
        while(S>U)
            resampledParticles=[resampledParticles,pf.filter(i).sampleSet(m)];
            U = U + M;
            k=k+1;
            %fprintf(1,'[ %d / %d] w:%6.4f\n ',m,k,pf.filter(i).sampleSet(m).w);
        end
    end
    
    pf.filter(i).sampleSet = resampledParticles;
    
    for k=1:N
        pf.filter(i).sampleSet(k).pose(1) = pf.filter(i).sampleSet(k).pose(1) + gsl_ran_gaussian(pf.resample_x);
        pf.filter(i).sampleSet(k).pose(2) = pf.filter(i).sampleSet(k).pose(2) + gsl_ran_gaussian(pf.resample_xDot);
        pf.filter(i).sampleSet(k).pose(3) = pf.filter(i).sampleSet(k).pose(3) + gsl_ran_gaussian(pf.resample_y);
        pf.filter(i).sampleSet(k).pose(4) = pf.filter(i).sampleSet(k).pose(4) + gsl_ran_gaussian(pf.resample_yDot);
    end

end% end function 

function [features] = transformToGlobal(features,robotPose)
%     [sinus, cosinus] =sincos(robotPose.pose(3));
%     for i=1:size(features,2)
%         x = features(i).x*cosinus - features(i).y*sinus;
%         y = features(i).x*sinus + features(i).y*cosinus;
%         features(i).x = x + robotPose.pose(1);
%         features(i).y = y + robotPose.pose(2);
%     end
end

function [x] = gsl_ran_flat(a,b)
    x = a + (b-a).*rand(1,1);
end

function [x] = gsl_ran_gaussian(a)
    % Generate values from a normal distribution with mean 1 and standard deviation 2.
    % r = 1 + 2.*randn(100,1);
    x = a*randn(1,1);
end

function [sinus,cosinus] = sincos(angle)
    sinus = sin(angle);
    cosinus = cos(angle);
end

function printFilterStats(pf, i, tempW, filename,T,ii)
    if ((myIsField(pf.filter(i), 'cov')) == 0)
        fprintf(1, ' PF, filter %d, ID %d w:%6.4f  sz:%d pose[%6.4f,%6.4f,%6.4f,%6.4f] valid:%d count:%d\n', ...
            i , pf.filter(i).targetID, ...
            tempW, size(pf.filter(i).sampleSet, 2) , ...
            pf.filter(i).hypothesis.pose(1) , ...
            pf.filter(i).hypothesis.pose(2) , ...
            pf.filter(i).hypothesis.pose(3) , ...
            pf.filter(i).hypothesis.pose(4),...
            pf.filter(i).validTrack,pf.filter(i).validityCounter);
    else
        fprintf(1, ' PF, filter %d, ID %d w:%6.4f  sz:%d pose[%6.4f,%6.4f,%6.4f,%6.4f] cov[[%6.4f,%6.4f,%6.4f,%6.4f] valid:%d count:%d\n', ...
            i , pf.filter(i).targetID, ...
            tempW, size(pf.filter(i).sampleSet, 2) , ...
            pf.filter(i).hypothesis.pose(1) , ...
            pf.filter(i).hypothesis.pose(2) , ...
            pf.filter(i).hypothesis.pose(3) , ...
            pf.filter(i).hypothesis.pose(4) , ...
            pf.filter(i).hypothesis.cov(1) , ...
            pf.filter(i).hypothesis.cov(2) , ...
            pf.filter(i).hypothesis.cov(3) , ...
            pf.filter(i).hypothesis.cov(4) ,...
            pf.filter(i).validTrack,pf.filter(i).validityCounter);
    end
    
    % Write to CSV file
if nargin == 6
    fileID = fopen(filename, 'a'); % Open the file for appending
    
    % Define header string
    headerString = 'TimeStamp,PF,ID,W,sz,x,vx,y,vy';
    if myIsField(pf.filter(i), 'cov')
        headerString = [headerString ',Cov1,Cov2,Cov3,Cov4'];
    end
    headerString = [headerString ',Valid,Count\n'];
    
    % Write header to file
    if ftell(fileID) == 0  % Check if file is empty
        fprintf(fileID, headerString);
    end
    
    % Write data
    if myIsField(pf.filter(i), 'cov') == 0
        fprintf(fileID, '%.4f,%d,%d,%6.4f,%d,%6.4f,%6.4f,%6.4f,%6.4f,%d,%d\n', ...
            T(ii), i, pf.filter(i).targetID, ...
            tempW, size(pf.filter(i).sampleSet, 2), ...
            pf.filter(i).hypothesis.pose(1), ...
            pf.filter(i).hypothesis.pose(2), ...
            pf.filter(i).hypothesis.pose(3), ...
            pf.filter(i).hypothesis.pose(4), ...
            pf.filter(i).validTrack, pf.filter(i).validityCounter);
    else
        fprintf(fileID, '%.4f,%d,%d,%6.4f,%d,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%d,%d\n', ...
            T(ii), i, pf.filter(i).targetID, ...
            tempW, size(pf.filter(i).sampleSet, 2), ...
            pf.filter(i).hypothesis.pose(1), ...
            pf.filter(i).hypothesis.pose(2), ...
            pf.filter(i).hypothesis.pose(3), ...
            pf.filter(i).hypothesis.pose(4), ...
            pf.filter(i).hypothesis.cov(1), ...
            pf.filter(i).hypothesis.cov(2), ...
            pf.filter(i).hypothesis.cov(3), ...
            pf.filter(i).hypothesis.cov(4), ...
            pf.filter(i).validTrack, pf.filter(i).validityCounter);
    end
    
    fclose(fileID); % Close the file
end
end

function [pf] = personRangeModel(pf,features)

    %variance= 2;

    %for all filters
    for i=1:size(pf.filter,2)

        if (pf.filter(i).featureIndex > -1)
            sum=0.; %for normalisation
            maxW=0;
            % for all samples of the i-th filter
            % std::cout << "in RM, FI:" << pf.filter(i)featureIndex << std::endl;
            for j=1:size(pf.filter(i).sampleSet,2)
                dx = features(pf.filter(i).featureIndex).x - pf.filter(i).sampleSet(j).pose(1);
                dy = features(pf.filter(i).featureIndex).y - pf.filter(i).sampleSet(j).pose(3);
                z  = sqrt(dx*dx+dy*dy);
                pf.filter(i).sampleSet(j).w = exp(-(z * z)/ (2 * pf.personRangeModel_variance * pf.personRangeModel_variance));
                if (pf.filter(i).sampleSet(j).w > maxW)
                    maxW=pf.filter(i).sampleSet(j).w;
                end
%                 fprintf(1,'filter:%d, sample%d, dist:%6.4f w:%6.4f\n',i,j,z,pf.filter(i).sampleSet(j).w);               
                sum = sum + pf.filter(i).sampleSet(j).w;
            end
%             fprintf(1,'filter:%d, maxW %6.4f sum %6.4f\n',i,maxW,sum);   
%             fprintf(1,'Sum is %6.4f\n',sum);
            for j=1:size(pf.filter(i).sampleSet,2)
                pf.filter(i).sampleSet(j).w= pf.filter(i).sampleSet(j).w /sum;
            end
        else
            %When no feature observed then questions is what to do, sum is zero?
            %a) can be 1/n
            %b) can be against an artificial feature, propagated forward
            equWeights = 1/ size(pf.filter(i).sampleSet,2);
            %         //std::cout << "Resampling uniformly w:" << equWeights << std::endl;
            for j=1:size(pf.filter(i).sampleSet,2)
               pf.filter(i).sampleSet(j).w = equWeights;
            end
            %Total weight check
            totalW=0;
            for j=1:size(pf.filter(i).sampleSet,2)
                totalW = totalW + pf.filter(i).sampleSet(j).w;
            end
%             fprintf(1,'Total weight:%d\n',totalW);
        end
    end
end %end function

function isFieldResult = myIsField (inStruct, fieldName)
    % inStruct is the name of the structure or an array of structures to search
    % fieldName is the name of the field for which the function searches
    isFieldResult = 0;
    f = fieldnames(inStruct(1));
    for i=1:length(f)
        if(strcmp(f{i},strtrim(fieldName)))
            isFieldResult = 1;
            return;
        elseif isstruct(inStruct(1).(f{i}))
            isFieldResult = myIsField(inStruct(1).(f{i}), fieldName);
            if isFieldResult
                return;
            end
        end
    end
end

function printAllParticles(pf)
    
    if( (myIsField (pf, 'filter') ) ==0)
            return;
    end
    
    for i=1:size(pf.filter,2)
        for j=1:size(pf.filter(i).sampleSet,2)
            fprintf(1,'Filter %d, Particle %d pose [%6.4f,%6.4f,%6.4f,%6.4f]\n',i,j,pf.filter(i).sampleSet(j).pose);
        end
    end

end %end function