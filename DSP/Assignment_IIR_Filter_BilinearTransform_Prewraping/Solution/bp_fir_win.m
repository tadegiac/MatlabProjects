function [b] = bp_fir_win(f_L, f_U, N, f_S, win)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculates the coefficients of a N-th order FIR bandpass filter f
% for a given Bandwith B and center frequency f_0 [Hz] when using
% a sampling frequency of f_S [Hz].
% Parameter W (optinal) specifies the windowing function to be used
% (default: rectangular).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%Source def Blackman Window: https://www.sciencedirect.com/topics/engineering/blackman-window
if strcmp(win,'blackman')
    n=[-N/2:N/2];
    W=0.42+0.5*cos(2*pi.*abs(n)/N)+0.08*cos(4*pi.*abs(n)/N);
%     n=[0:N];
%     W=0.42-0.5*cos(2.*pi.*n./(N))+0.08*(4.*pi.*n./(N))
else
    W=ones(1,N+1);
end   
%% Lowpass
% fc=f_L/(f_S/2);
% b_tp=(sin(2*pi*fc*[-N/2:N/2]))./(pi*[-N/2:N/2]);
% b_tp(N/2+1)=2*fc; 
% figure;freqz(b_tp)
%% Highpass
%versuch2
% fc=f_U/(f_S/2);
% b_tp2=(sin(2*pi*fc*[-N/2:N/2]))./(pi*[-N/2:N/2]);
% b_tp2(N/2+1)=2*fc; 
% figure;freqz(b_tp2)
% b_hp=-b_tp2;
% b_hp(N/2+1)=1 - b_tp2(N/2+1)
% figure;freqz(b_hp)
%% Create a Bandpass (combine Low and Highpass)
% b_bp_1=b_tp+b_hp;
wc2=2*pi*f_U/(f_S);
wc1=2*pi*f_L/(f_S);
%for n!=0
b=1./(pi.*[-N/2:N/2]).*(sin(wc2.*[-N/2:N/2])-(sin(wc1.*[-N/2:N/2])));
b(N/2+1)=(wc2-wc1)/pi;

%% apply window
b=b.*W;
end

