function varargout = testfig(varargin)
% TESTFIG MATLAB code for testfig.fig
%      TESTFIG, by itself, creates a new TESTFIG or raises the existing
%      singleton*.
%
%      H = TESTFIG returns the handle to a new TESTFIG or the handle to
%      the existing singleton*.
%
%      TESTFIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TESTFIG.M with the given input arguments.
%
%      TESTFIG('Property','Value',...) creates a new TESTFIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before testfig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to testfig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help testfig

% Last Modified by GUIDE v2.5 01-Oct-2018 10:42:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @testfig_OpeningFcn, ...
                   'gui_OutputFcn',  @testfig_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before testfig is made visible.
function testfig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to testfig (see VARARGIN)

% Choose default command line output for testfig
handles.output = hObject;
fig = figure;
geoshow('landareas.shp', 'FaceColor', [0.5 1.0 0.5]);
handles.dcm_obj = datacursormode(fig);
set(handles.dcm_obj,'DisplayStyle','datatip',...
'SnapToDataVertex','off','Enable','on')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes testfig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = testfig_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function lat_Callback(hObject, eventdata, handles)
% hObject    handle to lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lat as text
%        str2double(get(hObject,'String')) returns contents of lat as a double


% --- Executes during object creation, after setting all properties.
function lat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lon_Callback(hObject, eventdata, handles)
% hObject    handle to lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lon as text
%        str2double(get(hObject,'String')) returns contents of lon as a double


% --- Executes during object creation, after setting all properties.
function lon_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function gmt_Callback(hObject, eventdata, handles)
% hObject    handle to gmt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gmt as text
%        str2double(get(hObject,'String')) returns contents of gmt as a double


% --- Executes during object creation, after setting all properties.
function gmt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gmt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

data;
latstr=get(handles.lat,'string');
lonstr=get(handles.lon,'string');
lat=str2double(latstr);
lon=str2double(lonstr);

con
% d(218).data;
for i=1:con
   [Az El] = SolarAzEl(d(i).data,lat,lon,0);
   azimuth(i)=Az;
   elevation(i)=El;
end
x1=azimuth; %theta s (azimuth)
x4=elevation ;%beta (elevation)
x2=deg2rad(180); %theta c
x3=deg2rad([1:1:90]); %summation angle
r=x4;
% length(r)
x1=deg2rad(x1);
x4=deg2rad(x4);
I=zeros(90,360);
final=zeros(1,12);
m=zeros(1,20);
x=zeros(1,20);
k=1;
s=zeros(359,89);
sum=0;
flag=1;
n=1;
%m=1/cos(degtorad(90-d(i))); %air mass ratio
p=.03; %reflectance

for i=1:360
    for j=1:90
        sum=0;
        for k=1:con
            if(mod(k-1,3)==0)
                n=n+5;
            end
            A=1160+75*sin((360/365)*(n-275));
            K=0.174+.035*sin((360/365)*(n-100));
            C=.095+.04*sin((360/365)*(n-100));
            m=1/cos(deg2rad(90-r(k)));
            x2=deg2rad(i);
    I=A*exp(-1*K*m)*(cos(x4(k))*cos(x1(k)-x2)*sin(x3(j))+sin(x4(k))*cos(x3(j))+C/2*(1+cos(x3(j)))+p/2*(sin(x4(k))+C)*(1+cos(x3(j))));
            sum=sum+I;
        end
        n=0;
        s(i,j)=sum;
        
    end
end
s(1,1)=2;
global s
for i=1:20
[maxValue, linearIndexesOfMaxes] = max(s(:));
maxs(i)=maxValue/con;
[rowsOfMaxes, colsOfMaxes] = find(s == maxValue);
final(i)=rowsOfMaxes;
x(i)=colsOfMaxes;
s(rowsOfMaxes, colsOfMaxes)=0;
end
set(handles.thetac,'string',x(1));
set(handles.equ,'string',final(1));
x;
final;

x_data=[46 38 25 13 5 7 7 9 19 31 42 47];
y_data=[171 167 162 150 102 58 76 132 163 174 176 175];
figure;
plot(x_data,'r-o')
hold on 
plot(y_data,'b-o')
legend('Tilt angle','Azimuth')
title('Angles of orientation')
xlabel('Time(months)');
ylabel('Angles(degree)');
grid on;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
c_info = getCursorInfo(handles.dcm_obj);
lat=c_info.Position(1,2);
lon=c_info.Position(1,1);
gmt=floor(lon/15);
% gmt=double2str(gmt);
set(handles.gmt,'string',gmt);
latstr=num2str(lat);
lonstr=num2str(lon);
set(handles.lat,'string',latstr);
set(handles.lon,'string',lonstr);