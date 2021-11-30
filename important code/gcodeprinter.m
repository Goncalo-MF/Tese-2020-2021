%liga usb tipo c a impressora e usa fprintf para escrever

fclose(usb); %garantir que não ficou aberto porque depois 
             % nao funciona so com o clear

clear, clc, close all

usb=serial('com8');
fopen(usb);
usb.baudrate=115200;


fprintf(usb,'G28');
fprintf(usb,'G90'); %goes to said position
fprintf(usb,'G0 Z160 F1000');
for t=1:10
    fprintf(usb,'G0 X90 F2000000');
    fprintf(usb,'G0 X70 F2000000');
    fprintf(usb,'G0 X90 F2000000');
    fprintf(usb,'G0 X80 F2000000');
    fprintf(usb,'G0 X98 F2000000');
    fprintf(usb,'G0 X90 F2000000');
    fprintf(usb,'G0 X80 F2000000');
    fprintf(usb,'G0 X90 F2000000');
end