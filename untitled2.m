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
Fb=get(handles.edit2,'Value');%����Ƶ��
t=(0:1/Fs:(N-1)/Fs);%����ʱ��
ab=get(handles.popupmenu1,'Value');%ͨ��
tc=get(handles.edit5,'Value');%�ο�ʱ��
tw=get(handles.edit7,'Value');%ת������
St=[t;S(:,ab)'];%��ȡ��ab������A:ab=1,B:ab=2
Sx=St(2,:);

%FFT�任
Y=fft(Sx,N);%��FFT�任
Ay=(abs(Y));%ȡģ
Ay=Ay/(N/2);%�����ʵ�ʵķ���
Ay(1)=Ay(1)/2;
F=([1:N]-1)*Fs/N;%��n����ʾ��Ƶ��


%��ƴ�ͨ�˲�����
%��˫���Ա任����б�ѩ��I�ʹ�ͨ�˲���
Wp=[Fb-2 Fb+2]*2*pi/Fs;%��һ��%ͨ����ֹƵ��,ת��Ϊ��Ƶ��
Ws=[Fb-4 Fb+4]*2*pi/Fs;%�����ֹƵ��
Rp=0.1;%ͨ�����Ƽ���<=0.1dB
Rs=10;%���˥������>=10dB
%�ڼ����˲���ģ���˲�������֮ǰ����ͨ�����������Ԥ���䣬���ģ���˲�����Ƶ�ʲ�����
Wap=2*Fs*tan(Wp/2);
Was=2*Fs*tan(Ws/2);
%��Wap��Wasȥ��ģ���˲����Ľ�����
[n,Wn]=cheb1ord(Wap,Was,Rp,Rs,'s');%�б�ѩ�����ѡ��
Bw=Wn(2)-Wn(1);%ת�����ͨ�˲�������
Wo=sqrt(Wn(1)*Wn(2));%ת�����ͨ�˲�������Ƶ��
[z,p,k]=cheb1ap(n,Rp);%ģ���б�ѩ���˲���������z,p,k�ֱ�Ϊ�˲�������㡢���㡢����
[A1,B1,C1,D1]=zp2ss(z,p,k);%�㼫��ģ��ת��Ϊ״̬����ģ��
[At,Bt,Ct,Dt]=lp2bp(A1,B1,C1,D1,Wo,Bw);%��ͨ�˲���ת��Ϊ��ͨ�˲���
[num1,den1]=ss2tf(At,Bt,Ct,Dt);%״̬����ģ��ת��Ϊ���ݺ���ģ��
[num,den]=bilinear(num1,den1,Fs);%˫���Ա任ת������
[H,W]=freqz(num,den,N,'whole');%ר����������ɢϵͳƵ�����Եĺ���freqz
f=W*Fs/(2*pi);%���ж�Ӧ��Ƶ��ת��

Sn=filter(num,den,Sx);%����Sx������ͨ�˲����Ժ���º���
SF=fft(Sn,N);%�Ե��Ӻ���Sn������ͨ�˲����Ժ���º�������fft�任
Ay2=abs(SF);%���ֵ
Ay2=Ay2/(N/2); %�����ʵ�ʵķ���
Ay2(1)=Ay2(1)/2;

%�����ȶ���ֵ����λ
[Ap,idxp] = findpeaks(Sn,'MinPeakHeight',1);%�󼫴�ֵ,ApΪ����ֵ��idxp��Ӧ����
Ax=[Ap;(idxp-1)/Fs];%��һͬ����t
td=find(Ax(2,:)>=tc);
xw=(Ax(2,td(1))-tc)/tw*360;
set(handles.edit6,'String',num2str(xw));
guidata(hObject,handles);
j=1;
for i=1:length(Ap)-1
    if abs(Ap(i)-Ap(i+1))<50%����50
       Amm(j)=Ap(i);%ƽ����ֵ
        j=j+1;
    end
end
Aa=mean(Amm);%ƽ�ȷ�ֵ��ֵ
set(handles.edit4,'String',num2str(Aa));
guidata(hObject,handles);


plot(handles.axes2,f(1:N/2),Ay2(1:N/2));%���Ƶ��Ӻ���S������ͨ�˲����Ժ��Ƶ��ͼ
xlabel(handles.axes2,'Ƶ��');
ylabel(handles.axes2,'����');
title(handles.axes2,'��ͨ�˲����Ƶ��ͼ');

plot(handles.axes3,t,Sn);%���Ƶ��Ӻ���S������ͨ�˲����Ժ��ʱ��ͼ��
xlabel(handles.axes3,'ʱ��');
ylabel(handles.axes3,'����');
title(handles.axes3,'�˲�����ź�'); 

plot(handles.axes4,f(1:N/2),max(Ay)*abs(H(1:N/2)),'r',F(1:N/2),Ay(1:N/2));%�����б�ѩ���ͨ�˲����ķ�Ƶ��Ӧͼ
legend(handles.axes4,'�б�ѩ��I�ʹ�ͨ�˲���','FFTƵ��ͼ','Location','Best');%������ʵ�λ�üӱ�ע
title(handles.axes4,'��ͨ�˲�����FFTƵ��ͼ'); 
xlabel(handles.axes4,'Ƶ��');
ylabel(handles.axes4,'����');
title(handles.axes4,'FFTƵ��ͼ���ͨ�˲���Ƶ����Ӧ����');

plot(handles.axes1,t,Sx);
title(handles.axes1,'�����ź�'); 
xlabel(handles.axes1,'ʱ��');
ylabel(handles.axes1,'����');



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
[m,n1]=size(S1);%S�Ĵ�С
%N1=m;%�����ǲ�������
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
[m,n1]=size(S1);%S�Ĵ�С
%N1=m;%�����ǲ�������
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
helpdlg({'���������ļ���ʽ�ǡ� *.txt ��','�ļ�����һ�� Nx3 �ľ��󣬵�һ����ͨ��A�ɼ����ݣ��ڶ�����ͨ��B',...
    '��������NΪ2^n(��2��)','�ο�ʱ�����ڼ�����λ'},'help')


% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox({'������˫���Ա任����Ƶ��б�ѩ��I�ʹ�ͨ�˲���','','10������ѧһ���˲���'...
    ,'��Ա��Ф�� ��� �쿡 Ҷ����  ������ ��ٚ  �Ƽ��� �����'},'����')



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
