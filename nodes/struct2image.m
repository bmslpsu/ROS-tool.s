function [frame,img] = struct2image(msg)
%% struct2image: converts ROS image stored in structure to matrix & ROS image object
%   INPUTS:
%       msg     :   input image structure containing the following fields
%                           Height
%                           Width
%                           Encoding
%                           IsBigendian
%                           Step
%                           Data
%   OUTPUTS:
%       frame 	:  output image matrix
%       img    	:  output image object
%

img                     = rosmessage(msg.MessageType);
img.Height              = msg.Height;
img.Width               = msg.Width;
img.Encoding            = msg.Encoding;
img.IsBigendian         = msg.IsBigendian;
img.Step                = msg.Step;
img.Data                = msg.Data;

% msg.Header.MessageType  = img_struct.Header.MessageType;
% msg.Header.Seq          = img_struct.Header.Seq;
% msg.Header.Stamp        = img_struct.Header.Stamp;
% msg.Header.FrameId      = img_struct.Header.FrameId;

frame = readImage(img);  
end