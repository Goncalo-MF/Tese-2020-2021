% liga o matlab a impressora pelo arduino
% matlab manda os valoes apos acao de controlo, printer devolve valores
% atuais, arduino serve como menssageiro

clear, clc, close all

LED_BUILTIN='D13';
deltablink=4;

port='COM4';
board='UNO';    

a=arduino(port,board);
%%
%loop exemplo

for k=1:10
    %turn off
    a.writeDigitalPin(LED_BUILTIN,0);
    pause(deltablink/2); %quarto de segundo

    %turn on
    a.writeDigitalPin(LED_BUILTIN,1  );
    pause(deltablink/2); %quarto de segundo


end

t = tcpclient("10.26.111.214",80)



M73 P0 R122
M201 X1250 Y1250 Z400 E5000 ; sets maximum accelerations, mm/sec^2
M203 X180 Y180 Z12 E80 ; sets maximum feedrates, mm/sec
M204 P1250 R1250 T1250 ; sets acceleration (P, T) and retract acceleration (R), mm/sec^2
M205 X8.00 Y8.00 Z2.00 E10.00 ; sets the jerk limits, mm/sec
M205 S0 T0 ; sets the minimum extruding and travel feed rate, mm/sec
M107 ; disable fan
G90 ; use absolute coordinates
M83 ; extruder relative mode
M92 E325 ; set steps/unit for extruder
M104 S170 ; set extruder temp for bed leveling
M140 S60 ; set bed temp
M109 R170 ; wait for bed leveling temp
M190 S60 ; wait for bed temp
G28 ; home all without mesh bed level
G29 ; mesh bed leveling 
M104 S215 ; set extruder temp
G92 E0.0
G1 Y-2.0 X179 F2400
G1 Z3 F720
M109 S215 ; wait for extruder temp

; intro line
G1 X170 F1000
G1 Z0.2 F720
G1 X110.0 E8.0 F900
M73 P0 R122
G1 X40.0 E10.0 F700
G92 E0.0

M221 S95 ; set flow
G21 ; set units to millimeters
G90 ; use absolute coordinates
M83 ; use relative distances for extrusion
M900 K0.2 ; Filament gcode
;BEFORE_LAYER_CHANGE
G92 E0.0
;0.2



