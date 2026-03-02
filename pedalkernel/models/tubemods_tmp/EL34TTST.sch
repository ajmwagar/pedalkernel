*version 6.3 710246329
u 573
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
pageloc 1 0 3593 
@status
n 0 96:09:27:13:23:04;846451384 e 
s 0 96:09:27:13:23:07;846451387 e 
*page 1 0 970 720 iA
@ports
port 2 AGND 195 260 h
@parts
part 5 VSRC 145 185 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG1
a 1 xp 9 0 32 24 hcn 100 REFDES=VG1
part 6 VSRC 260 185 h
a 0 x 0 0 0 0 hln 100 PKGREF=V_P
a 1 xp 9 0 -6 10 hcn 100 REFDES=V_P
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
part 56 VSRC 260 160 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VP
a 1 xp 9 0 22 38 hcn 100 REFDES=VP
part 248 VSRC 195 210 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VC
a 1 xp 9 0 28 24 hcn 100 REFDES=VC
part 361 VSRC 40 215 h
a 1 u 0 0 0 0 hcn 100 DC=350
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2NOM
a 1 xp 9 0 14 -4 hcn 100 REFDES=VG2NOM
part 384 EVALUE 60 215 h
a 0 u 13 0 -28 64 hln 100 EXPR=V(%IN+, %IN-)*T2+V(101)*T1
a 0 sp 11 0 8 42 hln 100 PART=EVALUE
a 0 a 0 0 0 0 hln 100 PKGREF=E2
a 1 ap 9 0 20 2 hln 100 REFDES=E2
part 500 VSRC 110 195 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2
a 1 xp 9 0 24 40 hcn 100 REFDES=VG2
part 569 EL34 195 160 h
a 0 sp 11 0 0 10 hlb 100 PART=EL34
a 0 x 0 0 0 0 hln 100 PKGREF=TU1
a 1 xp 9 0 18 -2 hln 100 REFDES=TU1
part 570 PARAM 55 115 h
a 0 u 13 0 0 20 hln 100 NAME1=TRIMODE
a 0 u 13 0 0 30 hln 100 NAME2=T1
a 0 u 13 0 0 40 hln 100 NAME3=T2
a 0 u 13 0 50 22 hlb 100 VALUE1=100
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
w 405
a 0 sr 3 0 197 115 hln 100 LABEL=1P
s 260 105 195 105 404
a 0 sr 3 0 197 125 hln 100 LABEL=1P
s 195 120 195 105 406
s 260 120 260 105 408
w 419
a 0 sr 3 0 240 174 hln 100 LABEL=101
s 260 185 260 160 517
a 0 sr 3 0 240 174 hln 100 LABEL=101
w 473
a 0 sr 3 0 147 175 hln 100 LABEL=1G
s 155 165 145 165 472
a 0 sr 3 0 149 175 hln 100 LABEL=1G
s 145 185 145 165 515
w 504
a 0 sr 0 0 0 0 hln 100 LABEL=2G
s 110 155 155 155 554
a 0 sr 3 0 148 153 hln 100 LABEL=2G
w 512
a 0 sr 3 0 197 207 hln 100 LABEL=1C
s 195 210 195 200 519
a 0 sr 3 0 197 203 hln 100 LABEL=1C
w 551
s 145 255 195 255 539
s 145 255 145 225 537
s 195 260 195 255 529
s 195 255 195 250 536
s 260 255 195 255 527
s 260 255 260 225 530
s 110 255 145 255 552
s 110 225 110 255 558
s 110 255 60 255 562
s 60 255 40 255 566
s 60 225 60 255 564
w 557
a 0 sr 3 0 90 198 hln 100 LABEL=102
s 110 195 110 215 556
a 0 sr 3 0 112 196 hln 100 LABEL=102
w 568
s 60 215 40 215 567
@junction
j 260 120
+ p 56 -
+ w 405
j 260 160
+ p 56 +
+ w 419
j 145 185
+ p 5 +
+ w 473
j 260 185
+ p 6 +
+ w 419
j 195 210
+ p 248 +
+ w 512
j 195 260
+ s 2
+ w 551
j 195 250
+ p 248 -
+ w 551
j 260 225
+ p 6 -
+ w 551
j 195 255
+ w 551
+ w 551
j 145 225
+ p 5 -
+ w 551
j 145 255
+ w 551
+ w 551
j 110 155
+ p 500 -
+ w 504
j 110 215
+ p 384 OUT+
+ w 557
j 110 195
+ p 500 +
+ w 557
j 110 225
+ p 384 OUT-
+ w 551
j 110 255
+ w 551
+ w 551
j 40 255
+ p 361 -
+ w 551
j 60 225
+ p 384 IN-
+ w 551
j 60 255
+ w 551
+ w 551
j 40 215
+ p 361 +
+ w 568
j 60 215
+ p 384 IN+
+ w 568
j 195 120
+ p 569 P
+ w 405
j 155 165
+ p 569 G
+ w 473
j 155 155
+ p 569 G2
+ w 504
j 195 200
+ p 569 C
+ w 512
@attributes
a 0 s 0 0 0 0 hln 100 PAGETITLE=
a 0 s 0 0 0 0 hln 100 PAGENO=1
a 0 s 0 0 0 0 hln 100 PAGESIZE=A
a 0 s 0 0 0 0 hln 100 PAGECOUNT=1
@text
s 5 10 95 hln 100 TRIMODE IS % MODE (0=PENTODE, 40=UL, 100=TRIODE).
