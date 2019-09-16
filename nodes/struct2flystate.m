function [flystate] = struct2flystate(msg)
%% struct2flystate: converts ROS image stored in structure to matrix & ROS image object
%   INPUTS:
%       msg         :   input image structure containing the following fields
%                           Header
%                           Head
%                           Abdomen
%                           Left
%                           Right
%                           Aux
%   OUTPUTS:
%       flystate 	:  output table
%

flystate = {msg.Header.Stamp.Sec, msg.Head.Angles, msg.Left.Angles, ...
            msg.Right.Angles , msg.Abdomen.Angles, msg.Aux.Angles};
eIdx = cellfun(@(x) isempty(x), flystate);
eIdx = find(eIdx==true);
for kk = 1:length(eIdx)
    flystate{eIdx(kk)} = nan;
end
flystate = cell2table(flystate);
flystate.Properties.VariableNames = {'Time','Head','Left','Right','Abdomen','WBF'};

% flystate.Time       = msg.Header.Stamp.Sec;
% flystate.Head       = msg.Head.Angles;
% flystate.Left       = msg.Left.Angles;
% flystate.Right      = msg.Right.Angles;
% flystate.Abdomen    = msg.Abdomen.Angles;
% flystate.WBF        = msg.Aux.Angles;

end