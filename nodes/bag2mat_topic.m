function [Vid,VidTime,FlyState,AI,FILES] = bag2mat_topic(varargin)
%% bag2mat_topic: Parses file name data and returns tables with relevant information: saves in .mat file
%   INPUTS:
%       root        :   varargin=root , root directory containg .bag files >>> files will be saved in a folder titled "mat"
%                       inside this directory. If no input is given, will default to current folder.
%   OUTPUTS:
%       Vid         :   raw video data
%       VidTime     :   normalized video time
%       FlyState  	:   fly kinematic data
%       AI          :   analog input voltages
%       FILES    	:   filename listing
%---------------------------------------------------------------------------------------------------------------------------------
%   USAGE:
%       [] = bag2mat()
%           - opens dialog window to select files in current folder
%       [] = bag2mat(root)
%           - opens dialog window to select files in user defined root folder
%       [] = bag2mat(root,'kinefly')
%           - for kinefly export RGB video
%---------------------------------------------------------------------------------------------------------------------------------
clear;clc
root = 'Q:\Box Sync\Research\bags\9-4-2019';
%---------------------------------------------------------------------------------------------------------------------------------
% % Allow user to set root directory
% if nargin==0
%     root = '';
% elseif nargin==1
%     root = varargin{1};
% else
%     error('DEBUG')
% end

% Set directory & get files
[FILES, PATH] = uigetfile({'*.bag', 'BAG-files'}, 'Select .bag files', root, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one file, store in cell
n.Files = length(FILES); % # of .bag files to parse

% Set output directory to store .mat files
matdir = [PATH 'mat']; % export directory to save .mat files
[status,~,~] = mkdir(matdir); % create directory for .mat files
if status
    %warning(msg)
    %disp(['Folder located: ' matdir])
else
    error('Directory not created')
end

ImageTopics = ["/camera/image_raw" ; "/kinefly/image_output"];
% n.Topic = length(TopicList); % # of topics in .bag files

% W = waitbar(0/n.Files,'Saving data...');
tic
for kk = 1:n.Files
 	clear Vid VidTime FlyState AI Time Msg Topic Bag syncTime
    % Get topics, messages, & time
    Bag         = rosbag([PATH FILES{kk}]); % load bag
    topics      = Bag.AvailableTopics.Properties.RowNames; % topic names
 	msgtypes    = string(Bag.AvailableTopics.MessageType); % topic types
    n.Topic     = size(topics,1); % of topics
    
    % Determine message types (image or struct) % rearrange so image comes first
    for jj = 1:length(ImageTopics)
        msgtypes(strcmp(topics,ImageTopics(jj))) = "CompressedImage";
    end
    msgtypes(~strcmp(msgtypes,"CompressedImage")) = 'struct';
    [msgtypes,topI] = sort(msgtypes);
    topics = topics(topI);
    
    disp('Extracting topics:')
    disp(topics)
    Msg     = cell(1,n.Topic); % messages for each topic
    Time  	= cell(1,n.Topic); % time for flystate & AI
    for jj = 1:n.Topic
        Topic = select(Bag, 'Topic', topics{jj}); % get topics
        Msg{jj} = readMessages(Topic,'DataFormat','struct'); % get messages
        Time{jj} = table2array(Topic.MessageList(:,1)); % get raw time
    end     
    
   	% Initialize variables
    n.Frame  	= length(Msg{1}); % # of video frames
    InitFrame = readImage(Msg{1}{1}); % first video frame
    [n.PixelY,n.PixelX,n.bit] = size(InitFrame); % size of first video frame
    Vid = uint8(nan(n.PixelY,n.PixelX,n.bit,n.Frame)); % video cell
    
    % Sync times
    syncTime        = Time{1}(1); % sync times to vid frame
    VidTime(:,1)    = Time{1} - syncTime; % video time

       
	% Store messages in cells
    for jj = 1:n.Frame % cycle through states
        
    end
    
%     % Put data in tables
%     FlyState = splitvars(table(FlyState));
%     FlyState.Properties.VariableNames = {'Time','Head','LWing','RWing','Abdomen','WBF'}; % fly state variables
%     if ~isempty(Msg{3})
%         AI = splitvars(table(AI));
%         chList = cell(n.ACh+1,1);
%         chList{1} = 'Time';
%         for jj = 1:n.ACh
%            chList{jj+1} = ['Ch' num2str(jj-1)];
%         end
%         AI.Properties.VariableNames = chList; % AI variables
%     else
%         AI = [];
%     end
    
%     % Save .mat file in directory
%     [~,filename,~] = fileparts(FILES{kk}); % get filename
%     dateIdx = strfind(filename,'201'); % will work until 2020
%     filename = filename(1:dateIdx-2); % remove date-time at end of filename   
%     save([PATH '\mat\' filename '.mat'] , 'Vid','VidTime','FlyState','AI','FILES','-v7.3') % save data to .mat file
%     waitbar(kk/n.Files,W,'Saving data...');
    
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