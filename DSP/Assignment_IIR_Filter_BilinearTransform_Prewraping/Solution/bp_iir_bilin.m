function [b,a,K] = bp_iir_bilin(f_L, f_U, f_S, pw)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculates the coefficients of a 2nd order IIR bandpass filter 
% for given lower and upper band edge frequencies [Hz] when using
% a bilinear transform for the analog prototype filter (1) with prewarping 
% (pw = 'prewarp')or without prewarping (else).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define Constants
f_0=sqrt(f_L*f_U);
B=f_U-f_L;
w_0=2*pi*f_0;
mu=B/(f_0*w_0);
A=1;%figure out what for
if strcmp(pw,'prewarp')
    v=(w_0)/(tan(w_0/(2*f_S)));
else
    v=2*f_S;
end   
%% Calculate Coefficients
K=(A*mu*v)/(1+(w_0^(-2))*v^2+mu*v);
%Nominator Coefficients
b_0=1;
b_1=0;
b_2=(-1);
%Denominator coefficients
a_0=1; 
a_1=(2*(1-(v^2*(w_0^(-2)))))/(1+(v^2*(w_0^(-2)))+mu*v);
a_2=(1-mu*v+(w_0^(-2))*v^2)/(1+(w_0^(-2))*v^2+mu*v);
%Returnvalues
b=[b_0 b_1 b_2];
a =[a_0 a_1 a_2];
end

