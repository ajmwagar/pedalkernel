*version 6.3 3569139109
u 662
TU? 6
V? 3
E? 3
PM? 2
@libraries
@analysis
.DC 1 0 0 0 1 1
+ 0 0 V_P
+ 0 4 0
+ 0 5 400
+ 0 6 5
.STEP 1 0 0
+ 0 VG1
+ 4 0
+ 5 -32
+ 6 -3.2
.OP 0 
.PROBE 0 0 1 1 0 3
@targets
@attributes
@translators
a 0 u 13 0 0 0 hln 100 PSPICE=PSPICE
a 0 u 13 0 0 0 hln 100 PCBOARDS=PCB
a 0 u 13 0 0 0 hln 100 CADSTAR=PCB
a 0 u 13 0 0 0 hln 100 EDIF=PCB
a 0 u 13 0 0 0 hln 100 ORCAD=PCB
a 0 u 13 0 0 0 hln 100 PADS=PCB
a 0 u 13 0 0 0 hln 100 PCAD=PCB
a 0 u 13 0 0 0 hln 100 PROTEL=PCB
a 0 u 13 0 0 0 hln 100 TANGO=PCB
a 0 u 13 0 0 0 hln 100 SCICARDS=PCB
a 0 u 13 0 0 0 hln 100 POLARIS=PCB
@index
pageloc 1 0 3598 
@status
n 0 96:09:27:13:20:48;846451248 e 
s 0 96:09:27:13:24:50;846451490 e 
*page 1 0 970 720 iA
@ports
port 2 AGND 215 270 h
@parts
part 5 VSRC 165 195 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG1
a 1 xp 9 0 32 24 hcn 100 REFDES=VG1
part 6 VSRC 280 195 h
a 0 x 0 0 0 0 hln 100 PKGREF=V_P
a 1 xp 9 0 -6 10 hcn 100 REFDES=V_P
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
part 56 VSRC 280 170 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VP
a 1 xp 9 0 22 38 hcn 100 REFDES=VP
part 248 VSRC 215 220 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VC
a 1 xp 9 0 28 24 hcn 100 REFDES=VC
part 361 VSRC 60 225 h
a 1 u 0 0 0 0 hcn 100 DC=350
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2NOM
a 1 xp 9 0 14 -4 hcn 100 REFDES=VG2NOM
part 384 EVALUE 80 225 h
a 0 u 13 0 -28 64 hln 100 EXPR=V(%IN+, %IN-)*T2+V(101)*T1
a 0 sp 11 0 8 42 hln 100 PART=EVALUE
a 0 a 0 0 0 0 hln 100 PKGREF=E2
a 1 ap 9 0 20 2 hln 100 REFDES=E2
part 500 VSRC 130 205 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2
a 1 xp 9 0 24 40 hcn 100 REFDES=VG2
part 569 EL34 215 170 h
a 0 sp 11 0 4 10 hlb 100 PART=EL34
a 0 x 0 0 0 0 hln 100 PKGREF=TU1
a 1 xp 9 0 14 -4 hln 100 REFDES=TU1
part 570 PARAM 60 120 h
a 0 u 13 0 0 20 hln 100 NAME1=TRIMODE
a 0 u 13 0 0 30 hln 100 NAME2=T1
a 0 u 13 0 0 40 hln 100 NAME3=T2
a 0 u 13 0 50 22 hlb 100 VALUE1=40
a 0 u 13 0 50 32 hlb 100 VALUE2={TRIMODE/100}
a 0 u 13 0 50 42 hlb 100 VALUE3={1-T1}
a 0 a 0 0 0 0 hln 100 PKGREF=PM1
a 1 ap 0 0 10 -2 hcn 100 refdes=PM1
part 0 titleblk 970 720 h
a 1 s 13 0 350 10 hcn 100 PageSize=A
a 1 s 13 0 180 60 hcn 100 PageTitle=
a 1 s 13 0 300 95 hrn 100 PageNO=1
a 1 s 13 0 340 95 hrn 100 PageCount=1
@conn
w 620
a 0 sr 3 0 217 135 hln 100 LABEL=1P
s 280 115 215 115 619
a 0 sr 3 0 217 135 hln 100 LABEL=1P
s 215 130 215 115 621
s 280 130 280 115 623
w 626
a 0 sr 3 0 260 184 hln 100 LABEL=101
s 280 195 280 170 625
a 0 sr 3 0 260 184 hln 100 LABEL=101
w 628
a 0 sr 3 0 169 185 hln 100 LABEL=1G
s 175 175 165 175 627
a 0 sr 3 0 169 185 hln 100 LABEL=1G
s 165 195 165 175 629
w 632
a 0 sr 3 0 168 163 hln 100 LABEL=2G
s 130 165 175 165 631
a 0 sr 3 0 168 163 hln 100 LABEL=2G
w 634
a 0 sr 3 0 217 213 hln 100 LABEL=1C
s 215 220 215 210 633
a 0 sr 3 0 217 213 hln 100 LABEL=1C
w 636
s 165 265 215 265 635
s 165 265 165 235 637
s 215 270 215 265 639
s 215 265 215 260 641
s 280 265 215 265 643
s 280 265 280 235 645
s 130 265 165 265 647
s 130 235 130 265 649
s 130 265 80 265 653
s 80 265 60 265 657
s 80 235 80 265 655
w 659
a 0 sr 3 0 132 206 hln 100 LABEL=102
s 130 205 130 225 658
a 0 sr 3 0 132 206 hln 100 LABEL=102
w 661
s 80 225 60 225 660
@junction
j 215 130
+ p 569 P
+ w 620
j 280 130
+ p 56 -
+ w 620
j 280 195
+ p 6 +
+ w 626
j 280 170
+ p 56 +
+ w 626
j 175 175
+ p 569 G
+ w 628
j 165 195
+ p 5 +
+ w 628
j 130 165
+ p 500 -
+ w 632
j 175 165
+ p 569 G2
+ w 632
j 215 220
+ p 248 +
+ w 634
j 215 210
+ p 569 C
+ w 634
j 165 235
+ p 5 -
+ w 636
j 215 270
+ s 2
+ w 636
j 215 260
+ p 248 -
+ w 636
j 215 265
+ w 636
+ w 636
j 280 235
+ p 6 -
+ w 636
j 165 265
+ w 636
+ w 636
j 130 235
+ p 384 OUT-
+ w 636
j 130 265
+ w 636
+ w 636
j 60 265
+ p 361 -
+ w 636
j 80 235
+ p 384 IN-
+ w 636
j 80 265
+ w 636
+ w 636
j 130 225
+ p 384 OUT+
+ w 659
j 130 205
+ p 500 +
+ w 659
j 60 225
+ p 361 +
+ w 661
j 80 225
+ p 384 IN+
+ w 661
@attributes
a 0 s 0 0 0 0 hln 100 PAGETITLE=
a 0 s 0 0 0 0 hln 100 PAGENO=1
a 0 s 0 0 0 0 hln 100 PAGESIZE=A
a 0 s 0 0 0 0 hln 100 PAGECOUNT=1
@text
s 5 20 100 hln 100 TRIMODE IS % MODE (0=PENTODE, 40=UL, 100=TRIODE).
