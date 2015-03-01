function varargout = untitled2(varargin)
% UNTITLED2 M-file for untitled2.fig
%      UNTITLED2, by itself, creates a new UNTITLED2 or raises the existing
%      singleton*.
%
%      H = UNTITLED2 returns the handle to a new UNTITLED2 or the handle to
%      the existing singleton*.
%
%      UNTITLED2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED2.M with the given input arguments.
%
%      UNTITLED2('Property','Value',...) creates a new UNTITLED2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled2

% Last Modified by GUIDE v2.5 29-Nov-2013 15:27:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled2_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled2_OutputFcn, ...
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


% --- Executes just before untitled2 is made visible.
function untitled2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled2 (see VARARGIN)

% Choose default command line output for untitled2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled2 wait for user response (see UIRESUME)
%uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled2_OutputFcn(hObject, eventdata, handles) 
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
Fs=get(handles.edit1,'Value');
N=str2double(get(handles.edit3,'String'));
S=str2num(get(handles.listbox1,'String'));
Fb=get(handles.edit2,'Value');%保留频率
t=(0:1/Fs:(N-1)/Fs);%采样时刻
ab=get(handles.popupmenu1,'Value');%通道
tc=get(handles.edit5,'Value');%参考时间
tw=get(handles.edit7,'Value');%转子周期
St=[t;S(:,ab)'];%读取第ab列数据A:ab=1,B:ab=2
Sx=St(2,:);

%FFT变换
Y=fft(Sx,N);%做FFT变换
Ay=(abs(Y));%取模
Ay=Ay/(N/2);%换算成实际的幅度
Ay(1)=Ay(1)/2;
F=([1:N]-1)*Fs/N;%点n所表示的频率


%设计带通滤波器：
%用双线性变换设计切比雪夫I型带通滤波器
Wp=[Fb-2 Fb+2]*2*pi/Fs;%归一化%通带截止频率,转换为角频率
Ws=[Fb-4 Fb+4]*2*pi/Fs;%阻带截止频率
Rp=0.1;%通带波纹假设<=0.1dB
Rs=10;%阻带衰减假设>=10dB
%在计算滤波器模拟滤波器阶数之前，对通带和阻带进行预畸变，求出模拟滤波器的频率参数：
Wap=2*Fs*tan(Wp/2);
Was=2*Fs*tan(Ws/2);
%用Wap和Was去求模拟滤波器的阶数。
[n,Wn]=cheb1ord(Wap,Was,Rp,Rs,'s');%切比雪夫阶数选择
Bw=Wn(2)-Wn(1);%转换后带通滤波器带宽
Wo=sqrt(Wn(1)*Wn(2));%转换后带通滤波器中心频率
[z,p,k]=cheb1ap(n,Rp);%模拟切比雪夫滤波器函数，z,p,k分别为滤波器的零点、极点、增益
[A1,B1,C1,D1]=zp2ss(z,p,k);%零极点模型转化为状态方程模型
[At,Bt,Ct,Dt]=lp2bp(A1,B1,C1,D1,Wo,Bw);%低通滤波器转换为带通滤波器
[num1,den1]=ss2tf(At,Bt,Ct,Dt);%状态方程模型转换为传递函数模型
[num,den]=bilinear(num1,den1,Fs);%双线性变换转换函数
[H,W]=freqz(num,den,N,'whole');%专门用于求离散系统频响特性的函数freqz
f=W*Fs/(2*pi);%进行对应的频率转换

Sn=filter(num,den,Sx);%函数Sx经过带通滤波器以后的新函数
SF=fft(Sn,N);%对叠加函数Sn经过带通滤波器以后的新函数进行fft变换
Ay2=abs(SF);%求幅值
Ay2=Ay2/(N/2); %换算成实际的幅度
Ay2(1)=Ay2(1)/2;

%计算稳定幅值和相位
[Ap,idxp] = findpeaks(Sn,'MinPeakHeight',1);%求极大值,Ap为极大值，idxp对应点数
Ax=[Ap;(idxp-1)/Fs];%减一同以上t
td=find(Ax(2,:)>=tc);
xw=(Ax(2,td(1))-tc)/tw*360;
set(handles.edit6,'String',num2str(xw));
guidata(hObject,handles);
j=1;
for i=1:length(Ap)-1
    if abs(Ap(i)-Ap(i+1))<50%精度50
       Amm(j)=Ap(i);%平稳数值
        j=j+1;
    end
end
Aa=mean(Amm);%平稳幅值均值
set(handles.edit4,'String',num2str(Aa));
guidata(hObject,handles);


plot(handles.axes2,f(1:N/2),Ay2(1:N/2));%绘制叠加函数S经过低通滤波器以后的频谱图
xlabel(handles.axes2,'频率');
ylabel(handles.axes2,'幅度');
title(handles.axes2,'带通滤波后的频谱图');

plot(handles.axes3,t,Sn);%绘制叠加函数S经过带通滤波器以后的时域图形
xlabel(handles.axes3,'时间');
ylabel(handles.axes3,'幅度');
title(handles.axes3,'滤波后的信号'); 

plot(handles.axes4,f(1:N/2),max(Ay)*abs(H(1:N/2)),'r',F(1:N/2),Ay(1:N/2));%绘制切比雪夫带通滤波器的幅频响应图
legend(handles.axes4,'切比雪夫I型带通滤波器','FFT频谱图','Location','Best');%在最合适的位置加标注
title(handles.axes4,'带通滤波器与FFT频谱图'); 
xlabel(handles.axes4,'频率');
ylabel(handles.axes4,'幅度');
title(handles.axes4,'FFT频谱图与带通滤波器频率响应曲线');

plot(handles.axes1,t,Sx);
title(handles.axes1,'输入信号'); 
xlabel(handles.axes1,'时间');
ylabel(handles.axes1,'幅度');



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
val=str2double(get(hObject,'String'));
if isempty(val)
    set(hObject,'String','0');
elseif (val>=0)
    set(hObject,'String',val);
else
    errordlg('Input must be a positive number','Error');
end
set(handles.edit1,'Value',val);
guidata(hObject,handles)

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



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
val=str2double(get(hObject,'String'));
if isempty(val)
    set(hObject,'String','0');
elseif (val>=0)
    set(hObject,'String',val);
else
    errordlg('Input must be a positive number','Error');
end
set(handles.edit2,'Value',val);
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
Val=get(hObject,'Value');
str=get(hObject,'String');
switch str{Val};
    case 'A'
        set(handles.popupmenu1,'Value',1);
        guidata(hObject,handles)
    case 'B'
        set(handles.popupmenu1,'Value',2);
        guidata(hObject,handles)
end


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.txt','Select the Data file'); 
file=fullfile(PathName,FileName); 
S1=load(file);
set(handles.listbox1,'String',num2str(S1));
guidata(hObject,handles);
[m,n1]=size(S1);%S的大小
%N1=m;%行数是采样点数
set(handles.edit3,'String',num2str(m));
guidata(hObject,handles);


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.
function pushbutton1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%N=handles.N;



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.txt','Select the Data file'); 
file=fullfile(PathName,FileName); 
S1=load(file);
set(handles.listbox1,'String',num2str(S1));
guidata(hObject,handles);
[m,n1]=size(S1);%S的大小
%N1=m;%行数是采样点数
set(handles.edit3,'String',num2str(m));
guidata(hObject,handles);

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton3.
function pushbutton3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton2.

% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton2.
function pushbutton2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)zx


% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
helpdlg({'导入数据文件格式是‘ *.txt ’','文件内是一组 Nx3 的矩阵，第一列是通道A采集数据，第二列是通道B',...
    '采样点数N为2^n(基2型)','参考时间用于计算相位'},'help')


% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox({'这是用双线性变换法设计的切比雪夫I型带通滤波器','','10工程力学一班滤波组'...
    ,'组员：肖洋 徐刚 徐俊 叶兆新  苏煜钊 张贇  黄吉林 刘璐瑶'},'关于')



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


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



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
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
val=str2double(get(hObject,'String'));
if isempty(val)
    set(hObject,'String','0');
elseif (val>=0)
    set(hObject,'String',val);
else
    errordlg('Input must be a positive number','Error');
end
set(handles.edit5,'Value',val);
guidata(hObject,handles)




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



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
val=str2double(get(hObject,'String'));
if isempty(val)
    set(hObject,'String','0');
elseif (val>=0)
    set(hObject,'String',val);
else
    errordlg('Input must be a  positive number','Error');
end
set(handles.edit7,'Value',val);
guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
