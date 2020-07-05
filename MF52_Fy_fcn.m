function Fy = MF52_Fy_fcn(A,X)
% This function completes the MF5.2 Fitting of tire data provided by the
% Tire Testing Consortium

global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY LTR LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY 
    
ALPHA  =  X(:,1)*pi/180;
Fz     =  abs(X(:,2));
GAMMA  =  X(:,3)*pi/180;

GAMMAy = GAMMA .* LGAY; %31 (%48 lgay=lg
Fz0PR  = FZ0  .*  LFZO; %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFz    = (Fz-Fz0PR) ./ Fz0PR; %14,  (%30)

% Setting initial parameters
PCy1    = A(1);
PDy1    = A(2);
PDy2    = A(3);
PDy3    = A(4);
PEy1    = A(5);
PEy2    = A(6);
PEy3    = A(7);
PEy4    = A(8);
PKy1    = A(9);
PKy2    = A(10);
PKy3    = A(11);
PHy1    = A(12);
PHy2    = A(13);
PHy3    = A(14);
PVy1    = A(15);
PVy2    = A(16);
PVy3    = A(17);
PVy4    = A(18);

SHy     = (PHy1+PHy2 .* DFz) .* LHY + PHy3 .* GAMMAy; %38,  (%55)
ALPHAy  = ALPHA+SHy;  %30 (%47)
Cy      = PCy1 .* LCY;  %32 (%49)
MUy     = (PDy1+PDy2 .* DFz) .* (1.0-PDy3 .* GAMMAy.^2) .* LMUY; %34 (%51)
Dy      = MUy .* Fz; %33 (%50)
KY      = PKy1 .* FZ0 .* sin(2.0 .* atan(Fz ./ (PKy2 .* FZ0 .* LFZO))) .* (1.0-PKy3 .* abs(GAMMAy)) .* LFZO .* LKY; %36 (%53)
By      = KY ./ (Cy .* Dy);  %37 (%54)
% NOTE, PER SVEN @TNO: "SIGN(ALPHAY)"IS CORRECT AS IN DOCUMENTATION & BELOW; IT'S NOT SUPPOSED TO BE "SIGN(GAMMAY)"
Ey      = (PEy1+PEy2 .* DFz) .* (1.0-(PEy3+PEy4 .* GAMMAy) .* sign(ALPHAy)) .* LEY; %35 (%52)
% NOTE: LVY MULTIPLIES ONLY PVY1&2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
SVy     = Fz .* ((PVy1+PVy2 .* DFz) .* LVY+(PVy3+PVy4 .* DFz) .* GAMMAy) .* LMUY; %39 (%56)
Fy0     = Dy .* sin(Cy .* atan(By .* ALPHAy-Ey .* (By .* ALPHAy-atan(By .* ALPHAy))))+SVy; %29 (%46)
Fy      = Fy0; %28
