function [Vid,FlyState,AI] = bag2mat(root)
%% bag2mat: parses file name data and returns tables with relevant information, saves in .mat file
%   INPUTS:
%       root        :   root directory containg .bag files >>> files will be saved in a folder titled "mat"
%                       inside this directory. If no input is given, will default to current folder.
%   OUTPUTS:
%       Vid         :   raw video data
%       FlyState  	:   fly kinematic data
%       AI          :   analog input voltages
%       VO          :   phidget output voltages
%---------------------------------------------------------------------------------------------------------------------------------
%   USAGE:
%       [] = bag2mat()
%           - opens dialog window to select files in current folder
%       [] = bag2mat(root)
%           - opens dialog window to select files in user defined root folder
%---------------------------------------------------------------------------------------------------------------------------------
% clear;clc
% root = 'E:\Walking_Experiments\SOS';
%---------------------------------------------------------------------------------------------------------------------------------
% Allow user to set root directory
if nargin==0
    root = '';
end

% Set directory & get files
[FILES, PATH] = uigetfile({'*.bag', 'BAG-files'}, 'Select .bag files', root, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one file, store in cell
n.Files = length(FILES); % # of .bag files to parse

% Set output directory to store .mat files
matdir = [PATH 'mat']; % export directory to save .mat files
[status,msg,~] = mkdir(matdir); % create directory for .mat files
if status
    warning(msg)
    disp(['Folder located: ' matdir])
else
    error('Directory not created')
end

TopicList = ["/camera/image_raw" ; "/kinefly/image_output" ; "/kinefly/flystate" ; "/mcdaq/AI" ; ...
             "/kinefly/flystate2phidgetsanalog/voltages" ; "/kinefly/flystate2phidgetsanalog/vcoeff"];
         
n.Topic = length(TopicList); % # of topics in .bag files

W = waitbar(0/n.Files,'Converting data...');
tic
for kk = 1:n.Files
 	clear Vid VidTime FlyState AI Time Msg Topic Bag syncTime
    fprintf('\nFile %i : %s \n', kk, FILES{kk})
    
    % Get topics, messages, & time
    Bag = rosbag([PATH FILES{kk}]); % load bag
    topics = Bag.AvailableTopics.Properties.RowNames; % topic names
    
    disp('Extracting topics:')
    disp(topics)
    
    Msg = cell(1,n.Topic); % messages for each topic
    Time = cell(1,n.Topic); % time for flystate & AI
    Data = struct('Topic', '', 'Msg', nan);
    for jj = 1:n.Topic
        Topic = select(Bag, 'Topic', TopicList(jj)); % get topics       
        Msg{jj} = readMessages(Topic,'DataFormat','struct'); % get messages
        Time{jj} = table2array(Topic.MessageList(:,1)); % get raw time
        Data(jj).Topic = TopicList(jj);
        try
            Data(jj).Msg = Msg{jj};
        catch
           warning('no data asociated with topic: %s', TopicList(jj)) 
        end
    end
    
	% Get video messages
    sync = Time{1}(1);
    Vid = struct('Topic', '', 'Data', nan, 'Time', nan);
    for jj = 1:2 % cycle through states
        Vid(jj).Topic = TopicList(jj);
        if ~isempty(Data(jj).Msg)
            Data(jj).Points = length(Data(jj).Msg);
            [frame,~] = struct2image(Data(jj).Msg{1});
            dim = size(frame);
            Vid(jj).Data = uint8(zeros(dim(1),dim(2),Data(jj).Points));
            for ii = 1:Data(jj).Points
                Vid(jj).Data(:,:,ii) = struct2image(Data(jj).Msg{ii});
            end
            Vid(jj).Time = Time{jj} - sync;
        else
            Vid(jj).Data = nan;
            Vid(jj).Time = nan;
        end
    end
    
	% Get flystate messages
    for jj = 3 % cycle through states
        if ~isempty(Data(jj).Msg)
            Data(jj).Points = length(Data(jj).Msg);
            FlyState = struct2flystate(Data(jj).Msg{1});
            for ii = 2:Data(jj).Points
                FlyState(ii,:) = struct2flystate(Data(jj).Msg{ii});
            end
            FlyState.Time = Time{jj} - sync;
        else
            FlyState = nan;
        end
    end
    
    % Get mcdaq AI messages
    for jj = 4 % cycle through states
        if ~isempty(Data(jj).Msg)
            Data(jj).Points = length(Data(jj).Msg);
            ch = Data(jj).Msg{1,1}.Channels;
            n.AI = max(ch) + 1;
            AI = nan(Data(jj).Points,n.AI);
            for ii = 1:Data(jj).Points
                AI(ii,ch+1) = Data(jj).Msg{ii}.Voltages;
            end
        	AI = splitvars(table(AI));
            AI.Properties.VariableNames = repmat("ch",n.AI,1) + string(ch);
            AI.Time = Time{jj} - sync;
        else
            rep = 10;
            AI = splitvars(table(nan(1,rep)));
            AI.Properties.VariableNames = repmat("ch",rep,1) + string((1:rep)'-1);
            AI.Time = nan;
        end
        AI = movevars(AI,'Time','Before','ch0');
    end
    
	% Get phidget output voltage messages
    for jj = 5 % cycle through states
        if ~isempty(Data(jj).Msg)
            Data(jj).Points = length(Data(jj).Msg);
            ch = Data(jj).Msg{1,1}.Channels;
            n.VO = max(ch) + 1;
            VO = nan(Data(jj).Points,n.AI);
            for ii = 1:Data(jj).Points
                VO(ii,ch+1) = Data(jj).Msg{ii}.Voltages;
            end
            VO = splitvars(table(VO));
            VO.Properties.VariableNames = repmat("ch",n.VO,1) + string(ch);
            VO.Time = Time{jj} - sync;
        else
            rep = 4;
            VO = splitvars(table(nan(1,rep)));
            VO.Properties.VariableNames = repmat("ch",rep,1) + string((1:rep)'-1);
            VO.Time = nan;
        end
        VO = movevars(VO,'Time','Before','ch0');
    end
    
%  	% Get phidget voltage coefficents
%     for jj = 6 % cycle through states
%         if ~isempty(Data(jj).Msg)
%             Data(jj).Points = length(Data(jj).Msg);
%             ch = Data(jj).Msg{1,1}.Channels;
%             n.VC = max(ch) + 1;
%             VC = nan(Data(jj).Points,n.AI);
%             for ii = 1:Data(jj).Points
%                 VC(ii,ch+1) = Data(jj).Msg{ii}.Voltages;
%             end
%            	VC = splitvars(table(VC));
%             VC.Properties.VariableNames = repmat("ch",n.VC,1) + string(ch);
%         end
%     end
       
    % Save .mat file in directory
    [~,filename,~] = fileparts(FILES{kk}); % get filename
    
    % Remove data from filename
    try
        dateIdx(1) = strfind(filename,'201'); % 2019
    catch
        dateIdx(1) = nan;
    end
    try
        dateIdx(2) = strfind(filename,'202'); % 2020
    catch
        dateIdx(2) = nan;
    end
    dateIdx = dateIdx(~isnan(dateIdx));
    filename = filename(1:dateIdx-2); % remove date-time at end of filename
    
    rawVid = Vid(1).Data;
    rawTime = Vid(1).Time;
  	kinVid = Vid(2).Data;
    kinTime = Vid(2).Time;
    
    % Save Data
    save([PATH '\mat\' filename '.mat'], 'rawVid', 'rawTime','kinVid', 'kinTime', 'FlyState','AI','VO','-v7.3') % save data to .mat file
    waitbar(kk/n.Files,W,'Converting data...');
end
close(W)
disp('DONE')
toc
beep on
for kk = 1:5
    beep
    pause(0.5)
end
end