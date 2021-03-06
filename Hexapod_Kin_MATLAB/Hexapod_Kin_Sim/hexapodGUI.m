
function varargout = hexapodGUI(varargin)
% HEXAPODGUI MATLAB code for hexapodGUI.fig
%      HEXAPODGUI, by itself, creates a new HEXAPODGUI or raises the existing
%      singleton*.
%
%      H = HEXAPODGUI returns the handle to a new HEXAPODGUI or the handle to
%      the existing singleton*.
%
%      HEXAPODGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HEXAPODGUI.M with the given input arguments.
%
%      HEXAPODGUI('Property','Value',...) creates a new HEXAPODGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before hexapodGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to hexapodGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help hexapodGUI

% Last Modified by GUIDE v2.5 27-Sep-2019 12:56:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @hexapodGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @hexapodGUI_OutputFcn, ...
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


% --- Executes just before hexapodGUI is made visible.
function hexapodGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to hexapodGUI (see VARARGIN)

% Choose default command line output for hexapodGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes hexapodGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = hexapodGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ================% MOVE BUTTON FUNCTION %================= %%

axes(handles.axes2) %Handle Axes2 as frame;

cla %Clear frame after every pushbutton action;

k = 0; %Auxliar Counter;

%Get Limite = Move number of frames;
limite = str2double(get(handles.edit17, 'String'));

%Get Desired Position Coordinates for the Control Point from text boxes;
xdes = str2double(get(handles.edit1, 'String'));
ydes = str2double(get(handles.edit4, 'String'));
zdes = str2double(get(handles.edit5, 'String'));

%Get Angle Shift
J = 0;

%Get Previous Value of Translation and Rotation;
Po = handles.P;
Rxo = handles.thetaRX;
Ryo = handles.thetaRY;
Rzo = handles.thetaRZ;

%Get the Sample Position in the Platform Coordinate System from the text boxes;
sampx = str2double(get(handles.edit20, 'String'));
sampy = str2double(get(handles.edit21, 'String'));
sampz = str2double(get(handles.edit22, 'String'));

%Check the desired rotation angles from the text boxes;
thetaRXdes = str2double(get(handles.edit14, 'String'));
thetaRYdes = str2double(get(handles.edit15, 'String'));
thetaRZdes = str2double(get(handles.edit16, 'String'));
    
%Apply the rotation matrix to the bring the Sample Coordinate to the Referece System;    
Samplerot = rotation(thetaRXdes,thetaRYdes,thetaRZdes);

%Calculate the Desired position Vector for the center of the platform;
Pcalc = [xdes; ydes; zdes] - Samplerot*[sampx; sampy; sampz];

%Config Handle Data;
guidata(hObject,handles)
       
%Define the X,Y,Z coordinates of the Center of the Platform;
x = Po(1); y = Po(2); z = Po(3);

%Define RX RY RZ:

thetaRX = Rxo; thetaRY = Ryo; thetaRZ = Rzo;
     
%Define P vector
P = [x; y; z];

%Define P increment
a = (Pcalc(1,1) - P(1))/limite;
b = (Pcalc(2,1) - P(2))/limite;
c = (Pcalc(3,1) - P(3))/limite;

%Calculate each angle increment
e = (thetaRXdes - thetaRX)/limite;
f = (thetaRYdes - thetaRY)/limite;
g = (thetaRZdes - thetaRZ)/limite; 

%Constructive Properties %========================================================================

Rground = 79.91;    % Ground Base of the Hexapod Radius;
Rbase = 49.659;     % Mobile Platform of the Hexapod Radius;

angleground = 23.47;    % Ground Base - Closest Angle between the joints;
anglebase = 50.03;      % Mobile Platform - Closest Angle between the joints;

%Calculation of the joints Angles for the definition of the base and platform vectors;
a_theta1 = 90 - (angleground/2);
a_theta2 = a_theta1 + angleground;
a_theta3 = a_theta2 + (360 - 3*angleground)/3;
a_theta4 = a_theta3 + angleground;
a_theta5 = a_theta4 + (360 - 3*angleground)/3;
a_theta6 = a_theta5 + angleground;

b_theta1 = 90 - (anglebase/2);
b_theta2 = b_theta1 + anglebase;
b_theta3 = b_theta2 + (360 - 3*anglebase)/3;
b_theta4 = b_theta3 + anglebase;
b_theta5 = b_theta4 + (360 - 3*anglebase)/3;
b_theta6 = b_theta5 + anglebase;

%Vector a1...a6 - Location of the ground joints in the reference coordinate system;

%Hexapod Position in the Space = [100; 100; 0];

a1 = Rground*[cosd(a_theta1+J); sind(a_theta1+J); 0] + [100; 100; 0];
a2 = Rground*[cosd(a_theta2+J); sind(a_theta2+J); 0] + [100; 100; 0];
a3 = Rground*[cosd(a_theta3+J); sind(a_theta3+J); 0] + [100; 100; 0];
a4 = Rground*[cosd(a_theta4+J); sind(a_theta4+J); 0] + [100; 100; 0];
a5 = Rground*[cosd(a_theta5+J); sind(a_theta5+J); 0] + [100; 100; 0];
a6 = Rground*[cosd(a_theta6+J); sind(a_theta6+J); 0] + [100; 100; 0];

%Vector b1...b6 - Location of the platform joints in the platform coordinate system;
b1 = Rbase*[cosd(b_theta1+J); sind(b_theta1+J); 0];
b2 = Rbase*[cosd(b_theta2+J); sind(b_theta2+J); 0];
b3 = Rbase*[cosd(b_theta3+J); sind(b_theta3+J); 0];
b4 = Rbase*[cosd(b_theta4+J); sind(b_theta4+J); 0];
b5 = Rbase*[cosd(b_theta5+J); sind(b_theta5+J); 0];
b6 = Rbase*[cosd(b_theta6+J); sind(b_theta6+J); 0];

%Display the Calculated Center of the Platform;
textLabel = sprintf(' %f', Pcalc(1,1));
set(handles.text36, 'String', textLabel);

textLabel = sprintf(' %f', Pcalc(2,1));
set(handles.text37, 'String', textLabel);

textLabel = sprintf(' %f', Pcalc(3,1));
set(handles.text38, 'String', textLabel);

%Animation While Loop %=============================================================================

%Check if User Inputs are out of range:

while k < (limite)
    
    error = 0;
    
    if (abs(Pcalc(1,1)- 100) > 20 ) || (abs(Pcalc(2,1) - 100) > 20) || (abs(Pcalc(3,1) - 108) > 10)
        
        error = 1;
        break 
    end
    
    if (abs(thetaRXdes) > 10) || (abs(thetaRYdes) > 10) || (abs(thetaRZdes) > 15)
        error = 2;
        break 
    end

%Kinematisc Calculations and Plot====================================================================
    
    hold off    
        
    k = k + 1; %Inc

    %Rotation Matrix:

    R = rotation(thetaRX,thetaRY,thetaRZ);

    %Hexapod Inverse Kinematics Equations %==========================================================

    S1 = P + R*b1 - a1;
    S2 = P + R*b2 - a2;
    S3 = P + R*b3 - a3;
    S4 = P + R*b4 - a4;
    S5 = P + R*b5 - a5;
    S6 = P + R*b6 - a6;

    %Vector Sum and Matrix to Vector Operations = Geometry Definitions %=============================

    t1 = a1.'; v1 = (S1.')+t1;
    t2 = a2.'; v2 = (S2.')+t2;
    t3 = a3.'; v3 = (S3.')+t3;
    t4 = a4.'; v4 = (S4.')+t4;
    t5 = a5.'; v5 = (S5.')+t5;
    t6 = a6.'; v6 = (S6.')+t6;

    vv1 = [t1;v1]; vv2 = [t2;v2]; vv3 = [t3;v3];
    vv4 = [t4;v4]; vv5 = [t5;v5]; vv6 = [t6;v6];

    %Plotting Starts %===============================================================================

    view(3);

    C = [100, 100, 0] ;   % center of circle 
    plot3(C(1),C(2),C(3),'*r')
    hold on

    %Ground Base Shape Vectors (Ground)

    t12 = [t1;t2]; t23 = [t2;t3]; t34 = [t3;t4];
    t45 = [t4;t5]; t56 = [t5;t6]; t61 = [t6;t1];

    %Platform Shape Vectors

    k12 = [v1;v2]; k23 = [v2;v3]; k34 = [v3;v4];
    k45 = [v4;v5]; k56 = [v5;v6]; k61 = [v6;v1];

    %Ground Base Shape Vectors Plot

    plot3(t12(:,1),t12(:,2),t12(:,3),'b','LineWidth',2);
    plot3(t23(:,1),t23(:,2),t23(:,3),'b','LineWidth',2);
    plot3(t34(:,1),t34(:,2),t34(:,3),'b','LineWidth',2);
    plot3(t45(:,1),t45(:,2),t45(:,3),'b','LineWidth',2);
    plot3(t56(:,1),t56(:,2),t56(:,3),'b','LineWidth',2);
    plot3(t61(:,1),t61(:,2),t61(:,3),'b','LineWidth',2);

    %Platform Shape Vectors PLot

    plot3(k12(:,1),k12(:,2),k12(:,3),'b','LineWidth',2);
    plot3(k23(:,1),k23(:,2),k23(:,3),'b','LineWidth',2);
    plot3(k34(:,1),k34(:,2),k34(:,3),'b','LineWidth',2);
    plot3(k45(:,1),k45(:,2),k45(:,3),'b','LineWidth',2);
    plot3(k56(:,1),k56(:,2),k56(:,3),'b','LineWidth',2);
    plot3(k61(:,1),k61(:,2),k61(:,3),'b','LineWidth',2);

    %Legs Vectors Plot
    plot3(vv1(:,1),vv1(:,2),vv1(:,3),'r','LineWidth',3); 
    plot3(vv2(:,1),vv2(:,2),vv2(:,3),'r','LineWidth',3);
    plot3(vv3(:,1),vv3(:,2),vv3(:,3),'r','LineWidth',3);
    plot3(vv4(:,1),vv4(:,2),vv4(:,3),'r','LineWidth',3);
    plot3(vv5(:,1),vv5(:,2),vv5(:,3),'r','LineWidth',3);
    plot3(vv6(:,1),vv6(:,2),vv6(:,3),'r','LineWidth',3);

    %Legs Calculations;
    L1 = norm(S1); L2 = norm(S2); L3 = norm(S3);
    L4 = norm(S4); L5 = norm(S5); L6 = norm(S6);

    %Display Legs Length;
    textLabel = sprintf('Leg 1   = %f', L1);
    set(handles.text24, 'String', textLabel);
    textLabel = sprintf('Leg 2    = %f', L2);
    set(handles.text15, 'String', textLabel);
    textLabel = sprintf('Leg 3   = %f', L3);
    set(handles.text20, 'String', textLabel);
    textLabel = sprintf('Leg 4    = %f', L4);
    set(handles.text21, 'String', textLabel);
    textLabel = sprintf('Leg 5    = %f', L5);
    set(handles.text22, 'String', textLabel);
    textLabel = sprintf('Leg 6    = %f', L6);
    set(handles.text23, 'String', textLabel);


    %Sample Vector Calculation;

    P1 = (P.') + [sampx sampy 0];

    B = R*[sampx; sampy; sampz] + P;

    B1 = (B.');

    bb1 = [P1;B1];

    %Center of the Platform Point
    plot3(P1(:,1),P1(:,2),P1(:,3),'*r');
    %text(P1(:,1),P1(:,2),P1(:,3),'P');

    %Plot Sample;
    plot3(bb1(:,1),bb1(:,2),bb1(:,3),'g','LineWidth',3); 
    text(B1(:,1),B1(:,2),B1(:,3),'  Sample');
    plot3(B1(:,1),B1(:,2),B1(:,3),'og','markerEdgeColor', 'black','markerFaceColor', [0 1 0]);
    
    %Display Sample Coordinates;
    textLabel = sprintf(' %f', B1);
    set(handles.text45, 'String', textLabel);

    %Plot Ground Joints and Leg Numbers;
	plot3(t1(:,1),t1(:,2),t1(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t1(:,1),t1(:,2),t1(:,3),'- 1');
	plot3(t2(:,1),t2(:,2),t2(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t2(:,1),t2(:,2),t2(:,3),'- 2');
	plot3(t3(:,1),t3(:,2),t3(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t3(:,1),t3(:,2),t3(:,3),'- 3');
	plot3(t4(:,1),t4(:,2),t4(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t4(:,1),t4(:,2),t4(:,3),'- 4');
	plot3(t5(:,1),t5(:,2),t5(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t5(:,1),t5(:,2),t5(:,3),'- 5');
	plot3(t6(:,1),t6(:,2),t6(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	text(t6(:,1),t6(:,2),t6(:,3),'- 6');

	%Plot Platform Joints;
	plot3(v1(:,1),v1(:,2),v1(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	plot3(v2(:,1),v2(:,2),v2(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	plot3(v3(:,1),v3(:,2),v3(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	plot3(v4(:,1),v4(:,2),v4(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	plot3(v5(:,1),v5(:,2),v5(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
	plot3(v6(:,1),v6(:,2),v6(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
    
    % Painting the Area of the Platform and Base;
    v = [t1(1,1),t2(1,1),t3(1,1),t4(1,1),t5(1,1),t6(1,1)];
	vv = [t1(1,2),t2(1,2),t3(1,2),t4(1,2),t5(1,2),t6(1,2)];
	vvv = [t1(1,3),t2(1,3),t3(1,3),t4(1,3),t5(1,3),t6(1,3)];

	h1 = fill3(v,vv,vvv,[0.4 0.8 1]);
	h1.FaceColor = [0.4 0.8 1];
	h1.FaceAlpha = 0.3;

	u = [v1(1,1),v2(1,1),v3(1,1),v4(1,1),v5(1,1),v6(1,1)];
	uu = [v1(1,2),v2(1,2),v3(1,2),v4(1,2),v5(1,2),v6(1,2)];
	uuu = [v1(1,3),v2(1,3),v3(1,3),v4(1,3),v5(1,3),v6(1,3)];

	h1 = fill3(u,uu,uuu,[0.4 0.8 1]);
	h1.FaceColor = [0.4 0.8 1];
	h1.FaceAlpha = 0.3;

    % Graph Plot Settings %=========================================================================

    grid on
    xlabel('X');
    ylabel('Z');
    zlabel('    Y');
    xlim([0 200])
    ylim([0 200])
    zlim([0 200])
    h = get(gca,'zlabel'); 
    set(h,'Rotation',0);
    ax = gca;
    ax.ZRuler.FirstCrossoverValue  = 0; % crossover with X axis
    ax.ZRuler.SecondCrossoverValue = 0; % crossover with Y axis
    ax.XAxis.Direction = 'reverse';
    zticks([50 100 150 200])
    zticklabels({'50','100','150','200'})

    pause(0.1); %Movement time

    % Movement Increment %=========================================================================
    P = P + [a;  b; c];
    thetaRX = e + thetaRX;
    thetaRY = f + thetaRY;
    thetaRZ = g + thetaRZ;

    %==============================================================================================

    % Finish Movement Animation %==================================================================
    if k > (limite - 2)
        
        P = Pcalc;
        thetaRX = thetaRXdes;
        thetaRY = thetaRYdes;
        thetaRZ = thetaRZdes;    
       
    end

    textLabel = sprintf('W A I T - Frame = %f', k);
    set(handles.text34, 'String', textLabel);

    
end

% END of the While Loop %===========================================================================

% End of Move Indications %=========================================================================

if (error ~= 1) && (error ~= 2)
    textLabel = sprintf('MOVE DONE');
    set(handles.text34, 'String', textLabel);
end

if (error == 1)
    textLabel = sprintf('TRANSLATION OUT OF RANGE');
    set(handles.text34, 'String', textLabel);
end

if (error == 2)
    textLabel = sprintf('ROTATION OUT OF RANGE');
    set(handles.text34, 'String', textLabel);
end

% Choose default command line output for countClicks
handles.output = hObject;


% Save Data for the Nexts Movements %========================================================
aux1 = thetaRX;
aux2 = thetaRY;
aux3 = thetaRZ;

handles.moved = 1;
handles.P = P;
handles.thetaRX = aux1;
handles.thetaRY = aux2;
handles.thetaRZ = aux3;

handles.xsamp = B(1,1);
handles.ysamp = B(2,1);
handles.zsamp = B(3,1);

% Update handles structure
guidata(hObject, handles);

%==========================================================================

%%      - Newton Method Trial Solve Hexapod Forward Kinematics -         %%
%=========================================================================%

%Vector a1...a6 - Location of the ground joints in the reference coordinate system;

a1 = a1 - [100; 100; 0];
a2 = a2 - [100; 100; 0];
a3 = a3 - [100; 100; 0];
a4 = a4 - [100; 100; 0];
a5 = a5 - [100; 100; 0];
a6 = a6 - [100; 100; 0];

%==========================================================================

%Inputs: L1,L2,L3,L4,L5,L6.

%Variables=================================================================
syms x y z rx ry rz  

%==========================================================================

P = [x; y; z];
R = rotation2(rx,ry,rz);

%System====================================================================
eq1 = norm(P + R*b1 - a1) - L1;
eq2 = norm(P + R*b2 - a2) - L2;
eq3 = norm(P + R*b3 - a3) - L3;
eq4 = norm(P + R*b4 - a4) - L4;
eq5 = norm(P + R*b5 - a5) - L5;
eq6 = norm(P + R*b6 - a6) - L6;
%==========================================================================

%Newton Method=============================================================

% Forward Kinematics Initial Guess Calculations with Linear Regression %===

% Linear Regression Coef. Matrix (W0 + W1*L1 + W2*L2...+W6*L6)

RegCoef = [ 1.88740566 -1.88740566 -1.78630476e-1 1.70265653 -1.70265653 1.7863047e-1;
            -0.87999987 -0.87999987 2.07628491 -1.19622617 -1.19622617 2.07628491;
            0.17310112 0.17310112 0.1730323 0.17318892 0.17318892 0.1730323;
            0.09379136 0.09379136 0.60417547 -0.69795643 -0.69795643 0.60417547;
            -7.47210598e-1 7.47210598e-1 4.53910209e-1 -2.92504249e-1 2.92504249e-1 -4.53910209e-1;
            -1.22550406 1.22550406 -1.22207852 1.22116327 -1.22116327 1.22207852];

W0 = [-1.51694257e-9; -0.00958264; -11.31748409; -0.0012806; 6.14411133e-10; -3.11851055e-10];
            
L = [L1;L2;L3;L4;L5;L6];

% Initial Guess X0
X0 = RegCoef*L + W0;

% =========================================================================

%Aux:
del = 1;
index = 0;

%Desired Error:
Error = 1e-10;

F = [eq1; eq2; eq3; eq4; eq5; eq6];

J = jacobian(F,[x y z rx ry rz]);

f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));

%Newton Iteration Loop:
while del > Error
   
   index = index + 1;
   
   SubJacob = vpa(subs(J,{x,y,z,rx,ry,rz},{X0.'}));
     
   InverseJacob = inv(SubJacob);  
   
   delx = -InverseJacob*f;
     
   X0 = X0 + delx;
   
   f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));
   
   del = max(abs(f));
   
end

Result = X0 + [100; 100; 0; 0; 0; 0];

Result_T = [Result(1,1);Result(2,1);Result(3,1)] + Samplerot*[sampx; sampy; sampz];

errorX = xdes - (Result_T(1,1)); errorY = ydes - (Result_T(2,1)); errorZ = zdes - (Result_T(3,1));
errorRX = thetaRXdes - (Result(4,1)); errorRY = thetaRYdes - (Result(5,1)); errorRZ = thetaRZdes - (Result(6,1));

errorvector = [errorX errorY errorZ errorRX errorRY errorRZ];

textLabel = sprintf(' X = %f', Result_T(1,1));
set(handles.text57, 'String', textLabel);

textLabel = sprintf(' Z = %f', Result_T(2,1));
set(handles.text52, 'String', textLabel);

textLabel = sprintf(' Y = %f', Result_T(3,1));
set(handles.text53, 'String', textLabel);

textLabel = sprintf(' RX = %f', Result(4,1));
set(handles.text54, 'String', textLabel);

textLabel = sprintf(' RZ = %f', Result(5,1));
set(handles.text55, 'String', textLabel);

textLabel = sprintf(' RY = %f', Result(6,1));
set(handles.text56, 'String', textLabel);

MaxError = max(abs(errorvector));

textLabel = sprintf(' %f', MaxError);
set(handles.text61, 'String', textLabel);

textLabel = sprintf(' %f', errorvector);
set(handles.text59, 'String', textLabel);

textLabel = sprintf(' %i', index);
set(handles.text64, 'String', textLabel);

%============================================================================================

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

%if isnan(edit1)
 % set(handles.edit1,'string','10');
%end

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ====================%% START BUTTON %%==========================%%

% Same Code but only for reseting and get back for position Zero depending
% on the pushbutton 2

axes(handles.axes2) 

% Set Interface Default Values %

set(handles.edit1, 'string', '100');
set(handles.edit4, 'string', '100');
set(handles.edit5, 'string', '108');
set(handles.edit14, 'string', '0');
set(handles.edit15, 'string', '0');
set(handles.edit16, 'string', '0');
set(handles.edit17, 'string', '30');
set(handles.edit20, 'string', '0');
set(handles.edit21, 'string', '0');
set(handles.edit22, 'string', '0');
    
textLabel = sprintf(' %f', 100);
set(handles.text36, 'String', textLabel);

textLabel = sprintf(' %f', 100);
set(handles.text37, 'String', textLabel);

textLabel = sprintf(' %f', 108);
set(handles.text38, 'String', textLabel);
   
    
% Choose default command line output for countClicks
handles.output = hObject;

% Deafult Translation and Rotation Values;

handles.P = [100; 100; 108];

handles.thetaRX = 0;
handles.thetaRY = 0;
handles.thetaRZ = 0;

% Update handles structure
guidata(hObject, handles);

pause(0.1);

% Initial Settings:

cla

x = 100;
y = 100;
z = 108;

P = [x; y; z];

hold off    
    
% Constructive Properties %================================================

Rground = 79.91;
Rbase = 49.659;

angleground = 23.47;
anglebase = 50.03;

J = 0;

a_theta1 = 90 - (angleground/2);
a_theta2 = a_theta1 + angleground;
a_theta3 = a_theta2 + (360 - 3*angleground)/3;
a_theta4 = a_theta3 + angleground;
a_theta5 = a_theta4 + (360 - 3*angleground)/3;
a_theta6 = a_theta5 + angleground;

a1 = Rground*[cosd(a_theta1+J); sind(a_theta1+J); 0] + [100; 100; 0];
a2 = Rground*[cosd(a_theta2+J); sind(a_theta2+J); 0] + [100; 100; 0];
a3 = Rground*[cosd(a_theta3+J); sind(a_theta3+J); 0] + [100; 100; 0];
a4 = Rground*[cosd(a_theta4+J); sind(a_theta4+J); 0] + [100; 100; 0];
a5 = Rground*[cosd(a_theta5+J); sind(a_theta5+J); 0] + [100; 100; 0];
a6 = Rground*[cosd(a_theta6+J); sind(a_theta6+J); 0] + [100; 100; 0];

b_theta1 = 90 - (anglebase/2);
b_theta2 = b_theta1 + anglebase;
b_theta3 = b_theta2 + (360 - 3*anglebase)/3;
b_theta4 = b_theta3 + anglebase;
b_theta5 = b_theta4 + (360 - 3*anglebase)/3;
b_theta6 = b_theta5 + anglebase;

b1 = Rbase*[cosd(b_theta1+J); sind(b_theta1+J); 0];
b2 = Rbase*[cosd(b_theta2+J); sind(b_theta2+J); 0];
b3 = Rbase*[cosd(b_theta3+J); sind(b_theta3+J); 0];
b4 = Rbase*[cosd(b_theta4+J); sind(b_theta4+J); 0];
b5 = Rbase*[cosd(b_theta5+J); sind(b_theta5+J); 0];
b6 = Rbase*[cosd(b_theta6+J); sind(b_theta6+J); 0];

%Sample Coord:
sampx = str2double(get(handles.edit20, 'String'));
sampy = str2double(get(handles.edit21, 'String'));
sampz = str2double(get(handles.edit22, 'String'));

% Rotation Matrix:

R0 = [1 0 0; 0 1 0; 0 0 1];
R = R0;

% Hexapod Inverse Kinematics Equations:

S1 = P + R*b1 - a1;
S2 = P + R*b2 - a2;
S3 = P + R*b3 - a3;
S4 = P + R*b4 - a4;
S5 = P + R*b5 - a5;
S6 = P + R*b6 - a6;

% Vector Sum and Matrix to Vector Operations:

t1 = a1.'; v1 = (S1.')+t1;
t2 = a2.'; v2 = (S2.')+t2;
t3 = a3.'; v3 = (S3.')+t3;
t4 = a4.'; v4 = (S4.')+t4;
t5 = a5.'; v5 = (S5.')+t5;
t6 = a6.'; v6 = (S6.')+t6;

vv1 = [t1;v1]; vv2 = [t2;v2]; vv3 = [t3;v3];
vv4 = [t4;v4]; vv5 = [t5;v5]; vv6 = [t6;v6];

% Plotting Starts:

view(3);

C = [100, 100, 0] ;   % center of circle 
plot3(C(1),C(2),C(3),'*r')
hold on

% Base Shape Vectors

t12 = [t1;t2]; t23 = [t2;t3]; t34 = [t3;t4];
t45 = [t4;t5]; t56 = [t5;t6]; t61 = [t6;t1];

%Platform Shape Vectors

k12 = [v1;v2]; k23 = [v2;v3]; k34 = [v3;v4];
k45 = [v4;v5]; k56 = [v5;v6]; k61 = [v6;v1];

% Base Shape Vectors Plot

plot3(t12(:,1),t12(:,2),t12(:,3),'b','LineWidth',2);
plot3(t23(:,1),t23(:,2),t23(:,3),'b','LineWidth',2);
plot3(t34(:,1),t34(:,2),t34(:,3),'b','LineWidth',2);
plot3(t45(:,1),t45(:,2),t45(:,3),'b','LineWidth',2);
plot3(t56(:,1),t56(:,2),t56(:,3),'b','LineWidth',2);
plot3(t61(:,1),t61(:,2),t61(:,3),'b','LineWidth',2);

% Platform Shape Vectors PLot

plot3(k12(:,1),k12(:,2),k12(:,3),'b','LineWidth',2);
plot3(k23(:,1),k23(:,2),k23(:,3),'b','LineWidth',2);
plot3(k34(:,1),k34(:,2),k34(:,3),'b','LineWidth',2);
plot3(k45(:,1),k45(:,2),k45(:,3),'b','LineWidth',2);
plot3(k56(:,1),k56(:,2),k56(:,3),'b','LineWidth',2);
plot3(k61(:,1),k61(:,2),k61(:,3),'b','LineWidth',2);

% Legs Vectors
plot3(vv1(:,1),vv1(:,2),vv1(:,3),'r','LineWidth',3); 
plot3(vv2(:,1),vv2(:,2),vv2(:,3),'r','LineWidth',3);
plot3(vv3(:,1),vv3(:,2),vv3(:,3),'r','LineWidth',3);
plot3(vv4(:,1),vv4(:,2),vv4(:,3),'r','LineWidth',3);
plot3(vv5(:,1),vv5(:,2),vv5(:,3),'r','LineWidth',3);
plot3(vv6(:,1),vv6(:,2),vv6(:,3),'r','LineWidth',3);

% P point and Sample plot

P1 = (P.')+[sampx sampy 0];

B = R*[sampx; sampy; sampz] + P;

B1 = B.';

bb1 = [P1;B1];

% Center of the Platform Point
plot3(P1(:,1),P1(:,2),P1(:,3),'*r');
%text(P1(:,1),P1(:,2),P1(:,3),'P');

plot3(bb1(:,1),bb1(:,2),bb1(:,3),'g','LineWidth',3); 
text(B1(:,1),B1(:,2),B1(:,3),'  Sample');
plot3(B1(:,1),B1(:,2),B1(:,3),'og','markerEdgeColor', 'black','markerFaceColor', [0 1 0]);

textLabel = sprintf(' %f', B1);
set(handles.text45, 'String', textLabel);

plot3(t1(:,1),t1(:,2),t1(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t1(:,1),t1(:,2),t1(:,3),'- 1');
plot3(t2(:,1),t2(:,2),t2(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t2(:,1),t2(:,2),t2(:,3),'- 2');
plot3(t3(:,1),t3(:,2),t3(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t3(:,1),t3(:,2),t3(:,3),'- 3');
plot3(t4(:,1),t4(:,2),t4(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t4(:,1),t4(:,2),t4(:,3),'- 4');
plot3(t5(:,1),t5(:,2),t5(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t5(:,1),t5(:,2),t5(:,3),'- 5');
plot3(t6(:,1),t6(:,2),t6(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
text(t6(:,1),t6(:,2),t6(:,3),'- 6');

plot3(v1(:,1),v1(:,2),v1(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
plot3(v2(:,1),v2(:,2),v2(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
plot3(v3(:,1),v3(:,2),v3(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
plot3(v4(:,1),v4(:,2),v4(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
plot3(v5(:,1),v5(:,2),v5(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);
plot3(v6(:,1),v6(:,2),v6(:,3),'ok','markerEdgeColor', 'black','markerFaceColor', [0 0 1]);

% Painting the Area of the Platform and Base;
v = [t1(1,1),t2(1,1),t3(1,1),t4(1,1),t5(1,1),t6(1,1)];
vv = [t1(1,2),t2(1,2),t3(1,2),t4(1,2),t5(1,2),t6(1,2)];
vvv = [t1(1,3),t2(1,3),t3(1,3),t4(1,3),t5(1,3),t6(1,3)];

h1 = fill3(v,vv,vvv,[0.4 0.8 1]);
h1.FaceColor = [0.4 0.8 1];
h1.FaceAlpha = 0.3;

u = [v1(1,1),v2(1,1),v3(1,1),v4(1,1),v5(1,1),v6(1,1)];
uu = [v1(1,2),v2(1,2),v3(1,2),v4(1,2),v5(1,2),v6(1,2)];
uuu = [v1(1,3),v2(1,3),v3(1,3),v4(1,3),v5(1,3),v6(1,3)];

h1 = fill3(u,uu,uuu,[0.4 0.8 1]);
h1.FaceColor = [0.4 0.8 1];
h1.FaceAlpha = 0.3;

% Plot Settings;

grid on
xlabel('X');
ylabel('Z');
zlabel('    Y');
xlim([0 200])
ylim([0 200])
zlim([0 200])
h = get(gca,'zlabel'); 
set(h,'Rotation',0);
ax = gca;
ax.ZRuler.FirstCrossoverValue  = 0; % crossover with X axis
ax.ZRuler.SecondCrossoverValue = 0; % crossover with Y axis
ax.XAxis.Direction = 'reverse';
zticks([50 100 150 200])
zticklabels({'50','100','150','200'})

% Legs

L1 = norm(S1); L2 = norm(S2); L3 = norm(S3);
L4 = norm(S4); L5 = norm(S5); L6 = norm(S6);

textLabel = sprintf('Leg 1   = %f', L1);
set(handles.text24, 'String', textLabel);
textLabel = sprintf('Leg 2    = %f', L2);
set(handles.text15, 'String', textLabel);
textLabel = sprintf('Leg 3   = %f', L3);
set(handles.text20, 'String', textLabel);
textLabel = sprintf('Leg 4    = %f', L4);
set(handles.text21, 'String', textLabel);
textLabel = sprintf('Leg 5    = %f', L5);
set(handles.text22, 'String', textLabel);
textLabel = sprintf('Leg 6    = %f', L6);
set(handles.text23, 'String', textLabel);

textLabel = sprintf('Zero Position');
set(handles.text34, 'String', textLabel);


%      - Newton Method Trial Solve Hexapod Forward Kinematics -         %%
%=========================================================================%

%Vector a1...a6 - Location of the ground joints in the reference coordinate system;

a1 = a1 - [100; 100; 0];
a2 = a2 - [100; 100; 0];
a3 = a3 - [100; 100; 0];
a4 = a4 - [100; 100; 0];
a5 = a5 - [100; 100; 0];
a6 = a6 - [100; 100; 0];

%==========================================================================

%Inputs: L1,L2,L3,L4,L5,L6.

%Variables=================================================================
syms x y z rx ry rz  

%==========================================================================

P = [x; y; z];
R = rotation2(rx,ry,rz);

%System====================================================================
eq1 = norm(P + R*b1 - a1) - L1;
eq2 = norm(P + R*b2 - a2) - L2;
eq3 = norm(P + R*b3 - a3) - L3;
eq4 = norm(P + R*b4 - a4) - L4;
eq5 = norm(P + R*b5 - a5) - L5;
eq6 = norm(P + R*b6 - a6) - L6;
%==========================================================================

%Newton Method=============================================================

% Forward Kinematics Initial Guess Calculations with Linear Regression %===

% Linear Regression Coef. Matrix (W0 + W1*L1 + W2*L2...+W6*L6)

RegCoef = [ 1.88740566 -1.88740566 -1.78630476e-1 1.70265653 -1.70265653 1.7863047e-1;
            -0.87999987 -0.87999987 2.07628491 -1.19622617 -1.19622617 2.07628491;
            0.17310112 0.17310112 0.1730323 0.17318892 0.17318892 0.1730323;
            0.09379136 0.09379136 0.60417547 -0.69795643 -0.69795643 0.60417547;
            -7.47210598e-1 7.47210598e-1 4.53910209e-1 -2.92504249e-1 2.92504249e-1 -4.53910209e-1;
            -1.22550406 1.22550406 -1.22207852 1.22116327 -1.22116327 1.22207852];

W0 = [-1.51694257e-9; -0.00958264; -11.31748409; -0.0012806; 6.14411133e-10; -3.11851055e-10];
            
L = [L1;L2;L3;L4;L5;L6];

% Initial Guess X0
X0 = RegCoef*L + W0;

% =========================================================================

%Aux:
del = 1;
index = 0;

%Desired Error:
Error = 1e-10;

F = [eq1; eq2; eq3; eq4; eq5; eq6];

J = jacobian(F,[x y z rx ry rz]);

f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));
 
%Newton Iteration Loop:
while del > Error
    
   index = index + 1;
          
   SubJacob = vpa(subs(J,{x,y,z,rx,ry,rz},{X0.'}));
     
   InverseJacob = inv(SubJacob);  
   
   delx = -InverseJacob*f;
     
   X0 = X0 + delx;
   
   f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));
   
   del = max(abs(f));
     
end

errorX = 100 - (X0(1)+100); errorY = 100 - (X0(2)+100); errorZ = 108 - X0(3);
errorRX = 0 - X0(4); errorRY = 0 - X0(5); errorRZ = 0 - X0(6);

errorvector = [errorX errorY errorZ errorRX errorRY errorRZ];

textLabel = sprintf(' X = %f', X0(1,1)+100);
set(handles.text57, 'String', textLabel);

textLabel = sprintf(' Z = %f', X0(2,1)+100);
set(handles.text52, 'String', textLabel);

textLabel = sprintf(' Y = %f', X0(3,1));
set(handles.text53, 'String', textLabel);

textLabel = sprintf(' RX = %f', X0(4,1));
set(handles.text54, 'String', textLabel);

textLabel = sprintf(' RZ = %f', X0(5,1));
set(handles.text55, 'String', textLabel);

textLabel = sprintf(' RY = %f', X0(6,1));
set(handles.text56, 'String', textLabel);

MaxError = max(abs(errorvector));

textLabel = sprintf(' %f', MaxError);
set(handles.text61, 'String', textLabel);

textLabel = sprintf(' %f', errorvector);
set(handles.text59, 'String', textLabel);

textLabel = sprintf(' %i', index);
set(handles.text64, 'String', textLabel);

function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double

%if isnan(edit4)
 % set(handles.edit4,'string','10');
%end

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double

%if isnan(edit5)
 % set(handles.edit5,'string','10');
%end

% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double

edit14 = str2double(get(hObject,'string')); 
if isnan(edit14) 
set(handles.edit1,'string','0'); 
end 

% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double

%if isnan(edit15)
 % set(handles.edit15,'string','0');
%end

% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double

%if isnan(edit16)
 % set(handles.edit16,'string','0');
%end


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double

%if isnan(edit17)
%  set(handles.edit17,'string','40');
%end


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
