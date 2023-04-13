                          %% Robot GUI
function varargout = Robot_gui(varargin)
% ROBOT_GUI MATLAB code for Robot_gui.fig
%      ROBOT_GUI, by itself, creates a new ROBOT_GUI or raises the existing
%      singleton*.
%
%      H = ROBOT_GUI returns the handle to a new ROBOT_GUI or the handle to
%      the existing singleton*.
%
%      ROBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT_GUI.M with the given input arguments.
%
%      ROBOT_GUI('Property','Value',...) creates a new ROBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Robot_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Robot_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Robot_gui

% Last Modified by GUIDE v2.5 03-Jun-2022 15:06:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Robot_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Robot_gui_OutputFcn, ...
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


% --- Executes just before Robot_gui is made visible.
function Robot_gui_OpeningFcn(hObject, eventdata, handles, varargin)

format short

%% setting initial value of thetas in opening windows

handles.Theta1.String=0;
handles.Theta2.String=0;
handles.Theta3.String=0;
handles.Theta4.String=0;
handles.Theta5.String=0;
handles.Theta6.String=0;
set(handles.Table,'Data',zeros(3,3)) % initial value of rotation matrix

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Robot_gui (see VARARGIN)

% Choose default command line output for Robot_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Robot_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%% Define D-H Params 
    a2=10; d1=10; a3=15; d6=7;
    DH_Parameters=[0 d1 0  pi/2
                   0 0  a2 0
                   0 0  a3 0
                   0 0  0  -pi/2
                   0 0  0  pi/2
                   0 d6 0  0];

 
 Robot=RobotConfig(DH_Parameters); % Robot Configurated with D-H params
 Robot.plot(DH_Parameters(:,1)') % plot Robot with initial variables
 [H,~]=DH(DH_Parameters); % Calculate Homogenous matrix
 
 %% extract end-effector position and putting up them 
 handles.Xe.String=H(1,end);
 handles.Ye.String=H(2,end);
 handles.Ze.String=H(3,end);
 %% extract Rotation matrix from H
 handles.Table.Data=round(H(1:3,1:3),5);


% --- Outputs from this function are returned to the command line.
function varargout = Robot_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta1 as text
%        str2double(get(hObject,'String')) returns contents of Theta1 as a double


% --- Executes during object creation, after setting all properties.
function Theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta2 as text
%        str2double(get(hObject,'String')) returns contents of Theta2 as a double


% --- Executes during object creation, after setting all properties.
function Theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta5 as text
%        str2double(get(hObject,'String')) returns contents of Theta5 as a double


% --- Executes during object creation, after setting all properties.
function Theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta3 as text
%        str2double(get(hObject,'String')) returns contents of Theta3 as a double


% --- Executes during object creation, after setting all properties.
function Theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta4 as text
%        str2double(get(hObject,'String')) returns contents of Theta4 as a double


% --- Executes during object creation, after setting all properties.
function Theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta6 as text
%        str2double(get(hObject,'String')) returns contents of Theta6 as a double


% --- Executes during object creation, after setting all properties.
function Theta6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ze_Callback(hObject, eventdata, handles)
% hObject    handle to Ze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ze as text
%        str2double(get(hObject,'String')) returns contents of Ze as a double


% --- Executes during object creation, after setting all properties.
function Ze_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ye_Callback(hObject, eventdata, handles)
% hObject    handle to Ye (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ye as text
%        str2double(get(hObject,'String')) returns contents of Ye as a double


% --- Executes during object creation, after setting all properties.
function Ye_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ye (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Xe_Callback(hObject, eventdata, handles)
% hObject    handle to Xe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Xe as text
%        str2double(get(hObject,'String')) returns contents of Xe as a double


% --- Executes during object creation, after setting all properties.
function Xe_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Xe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

n=20;
%% read thetas from GUI
Theta_1=str2double(handles.Theta1.String)*pi/180;

Theta_2=str2double(handles.Theta2.String)*pi/180;

Theta_3=str2double(handles.Theta3.String)*pi/180;

Theta_4=str2double(handles.Theta4.String)*pi/180;

Theta_5=str2double(handles.Theta5.String)*pi/180;

Theta_6=str2double(handles.Theta6.String)*pi/180;
%% read rotation matrix
R=str2double(handles.Table.Data);

%% Theta change ranges
th{1}=linspace(0,Theta_1,n);

th{2}=linspace(0,Theta_2,n);

th{3}=linspace(0,Theta_3,n);

th{4}=linspace(0,Theta_4,n);

th{5}=linspace(0,Theta_5,n);

th{6}=linspace(0,Theta_6,n);


%% DH Parameters

               a2=10; d1=10; a3=15; d6=7;
DH_Parameters=[Theta_1   d1  0   pi/2
               Theta_2   0   a2  0
               Theta_3   0   a3  0
               Theta_4   0   0   -pi/2
               Theta_5   0   0   pi/2
               Theta_6   d6  0   0];
           
              

[H,~]=DH(DH_Parameters); % calculate H with new thetas

%% extract end-effecto position and orientation and putting up them
handles.Xe.String=num2str(round(H(1,end),5));
handles.Ye.String=num2str(round(H(2,end),5));
handles.Ze.String=num2str(round(H(3,end),5));

set(handles.Table,'Data',round(H(1:3,1:3),5))

Robot=RobotConfig(DH_Parameters); % call Robot

% plot robot 
for i=1:20
    
    Robot.plot([th{1}(i), th{2}(i), th{3}(i), th{4}(i), th{5}(i), th{6}(i)])
%     pause(.025)
    
end

% --- Executes on button press in inverse.
function inverse_Callback(hObject, eventdata, handles)
% hObject    handle to inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% read position and orientation
X=str2double(handles.Xe.String);
Y=str2double(handles.Ye.String);
Z=str2double(handles.Ze.String);
R=get(handles.Table, 'Data');
% define D-H params
    a2=10; d1=10; a3=15; d6=7;
    d=0;
    DH_Parameters=[0 d1 0 pi/2
                   0 0 a2 0
                   0 0 a3 0
                   0 0 0 -pi/2
                   0 0 0 pi/2
                   0 d6 0 0];

H60=[R(1,:),  X;
     R(2,:),  Y;
     R(3,:),  Z;
     0, 0, 0, 1];
H60=round(H60,5);  % Homogenous matrix


 Robot=RobotConfig(DH_Parameters); % call Robot
 
 Q=RRR_ikine(H60);  % solve inverse kinematics problem
 %% range of thetas
 Q1=linspace(0,Q(1),20);
 Q2=linspace(0,Q(2),20);
 Q3=linspace(0,Q(3),20);
 Q4=linspace(0,Q(4),20);
 Q5=linspace(0,Q(5),20);
 Q6=linspace(0,Q(6),20);
 q=Q*180/pi; % radian to degree
 %% upload thetas in GUI
 handles.Theta1.String=num2str(round(q(1),5));
 handles.Theta2.String=num2str(round(q(2),5));
 handles.Theta3.String=num2str(round(q(3),5));
 handles.Theta4.String=num2str(round(q(4),5));
 handles.Theta5.String=num2str(round(q(5),5));
 handles.Theta6.String=num2str(round(q(6),5));
 % Robot plot
 for i=1:20
    Robot.plot([Q1(i) Q2(i) Q3(i) Q4(i) Q5(i) Q6(i)])
    pause(.025)
 end

% --- Executes on button press in home.
function home_Callback(hObject, eventdata, handles)
% hObject    handle to home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


n=20;
%% read current thetas (and change degree to radian)
Theta_1=str2double(handles.Theta1.String)*pi/180;
th1=linspace(0,Theta_1,n);

Theta_2=str2double(handles.Theta2.String)*pi/180;
th2=linspace(0,Theta_2,n);

Theta_3=str2double(handles.Theta3.String)*pi/180;
th3=linspace(0,Theta_3,n);

Theta_4=str2double(handles.Theta4.String)*pi/180;
th4=linspace(0,Theta_4,n);

Theta_5=str2double(handles.Theta5.String)*pi/180;
th5=linspace(0,Theta_5,n);

Theta_6=str2double(handles.Theta6.String)*pi/180;
th6=linspace(0,Theta_6,n);
%% define D-H params
    a2=10; d1=10; a3=15; d6=7;
    d=0;
    DH_Parameters=[0 d1 0 pi/2
                   0 0 a2 0
                   0 0 a3 0
                   0 0 0 -pi/2
                   0 0 0 pi/2
                   0 d6 0 0];

Robot=RobotConfig(DH_Parameters); % call Robot
[H,~]=DH(DH_Parameters); % Homogenous matrix
% plot Robot
for i=n:-1:1
    
    Robot.plot([th1(i) th2(i) th3(i) th4(i) th5(i) th6(i)])
    pause(.025)
    
end
%% set position and orientation 
% position
 handles.Xe.String=H(1,end);
 handles.Ye.String=H(2,end);
 handles.Ze.String=H(3,end);
 % orientation
 handles.Table.Data=round(H(1:3,1:3),5);

%% Home value
handles.Theta1.String=num2str(0);
handles.Theta2.String=num2str(0);
handles.Theta3.String=num2str(0);
handles.Theta4.String=num2str(0);
handles.Theta5.String=num2str(0);
handles.Theta6.String=num2str(0);


% --- Executes on button press in random.
function random_Callback(hObject, eventdata, handles)
% hObject    handle to random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% random position in workspace
Xe=50*rand-25;
Ye=50*rand-25;
Ze=40*rand-20;

%% random orientation (orthonormal 3*3 matrix)
k=orth(rand(3));
% k=k(:,1);

% th=2*pi*rand-pi;
% 
% H=R(th,'k',k);
%% H using random values
H = zeros(4, 4);
H(1:3,1:3)=k;    
H(1,end)=Xe;
H(2,end)=Ye;
H(3,end)=Ze;

%% show part of H in GUI
handles.Xe.String=Xe;
handles.Ye.String=Ye;
handles.Ze.String=Ze;
handles.Table.Data=H(1:3,1:3);


%% define D-H
    a2=10; d1=10; a3=15; d6=7;
    d=0;
    DH_Parameters=[0 d1 0 pi/2
                   0 0 a2 0
                   0 0 a3 0
                   0 0 0 -pi/2
                   0 0 0 pi/2
                   0 d6 0 0];


 Robot=RobotConfig(DH_Parameters); % call Robot
 
 
 Theta1=str2double(handles.Theta1.String)*pi/180;
 Theta2=str2double(handles.Theta2.String)*pi/180;
 Theta3=str2double(handles.Theta3.String)*pi/180;
 Theta4=str2double(handles.Theta4.String)*pi/180;
 Theta5=str2double(handles.Theta5.String)*pi/180;
 Theta6=str2double(handles.Theta6.String)*pi/180;

 
 
%% solve Inverse kinematics
 Q=RRR_ikine(H);
 Q1=linspace(Theta1,Q(1),20);
 Q2=linspace(Theta2,Q(2),20);
 Q3=linspace(Theta3,Q(3),20);
 Q4=linspace(Theta4,Q(4),20);
 Q5=linspace(Theta5,Q(5),20);
 Q6=linspace(Theta6,Q(6),20);
 q=Q*180/pi;
 
 %% show angles in GUI
 handles.Theta1.String=num2str(round(q(1),5));
 handles.Theta2.String=num2str(round(q(2),5));
 handles.Theta3.String=num2str(round(q(3),5));
 handles.Theta4.String=num2str(round(q(4),5));
 handles.Theta5.String=num2str(round(q(5),5));
 handles.Theta6.String=num2str(round(q(6),5));
 for i=1:20
    Robot.plot([Q1(i) Q2(i) Q3(i) Q4(i) Q5(i) Q6(i)])
    pause(.025)
 end

 
 