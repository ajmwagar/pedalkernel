*version 6.3 1283018902
lib C:\audio\tubeshar\tube.slb 215529251
u 752
TU? 7
V? 3
E? 3
PM? 2
@libraries
C:\AUDIO\TUBESHAR\TUBE  [.slb]
@analysis
.DC 1 0 0 0 1 1
+ 0 0 V_P
+ 0 4 0
+ 0 5 400
+ 0 6 5
.STEP 1 0 0
+ 0 VG1
+ 4 0
+ 5 -20
+ 6 -2
.OP 0 
.PROBE 0 0 1 1 0 3
.LIB C:\AUDIO\TUBESHAR\TUBE97.LIB
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
pageloc 1 0 3628 
@status
n 0 97:10:29:23:42:19;880875739 e 
s 0 97:10:29:23:47:39;880876059 e 
*page 1 0 970 720 iA
@ports
port 2 AGND 205 265 h
@parts
part 5 VSRC 155 190 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG1
a 1 xp 9 0 32 24 hcn 100 REFDES=VG1
part 6 VSRC 270 190 h
a 0 x 0 0 0 0 hln 100 PKGREF=V_P
a 1 xp 9 0 -6 10 hcn 100 REFDES=V_P
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
part 56 VSRC 270 165 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VP
a 1 xp 9 0 22 38 hcn 100 REFDES=VP
part 248 VSRC 205 215 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VC
a 1 xp 9 0 28 24 hcn 100 REFDES=VC
part 361 VSRC 50 220 h
a 1 u 0 0 0 0 hcn 100 DC=200
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2NOM
a 1 xp 9 0 14 -4 hcn 100 REFDES=VG2NOM
part 384 EVALUE 70 220 h
a 0 u 13 0 -28 64 hln 100 EXPR=V(%IN+, %IN-)*T2+V(101)*T1
a 0 sp 11 0 8 42 hln 100 PART=EVALUE
a 0 a 0 0 0 0 hln 100 PKGREF=E2
a 1 ap 9 0 20 2 hln 100 REFDES=E2
part 500 VSRC 120 200 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG2
a 1 xp 9 0 24 40 hcn 100 REFDES=VG2
part 571 PARAM 55 115 h
a 0 u 13 0 0 20 hln 100 NAME1=TRIMODE
a 0 u 13 0 0 30 hln 100 NAME2=T1
a 0 u 13 0 0 40 hln 100 NAME3=T2
a 0 u 13 0 50 22 hlb 100 VALUE1=0
a 0 u 13 0 50 32 hlb 100 VALUE2={TRIMODE/100}
a 0 u 13 0 50 42 hlb 100 VALUE3={1-T1}
a 0 a 0 0 0 0 hln 100 PKGREF=PM1
a 1 ap 0 0 10 -2 hcn 100 refdes=PM1
part 751 6550 205 165 h
a 0 sp 11 0 0 10 hlb 100 PART=6550
a 0 a 0 0 0 0 hln 100 PKGREF=TU6
a 0 s 0 0 0 0 hln 100 PKGTYPE=
a 1 ap 9 0 -12 -6 hln 100 REFDES=TU6
part 0 titleblk 970 720 h
a 1 s 13 0 350 10 hcn 100 PageSize=A
a 1 s 13 0 180 60 hcn 100 PageTitle=
a 1 s 13 0 300 95 hrn 100 PageNO=1
a 1 s 13 0 340 95 hrn 100 PageCount=1
@conn
w 710
a 0 sr 3 0 207 130 hln 100 LABEL=1P
s 270 110 205 110 709
a 0 sr 3 0 207 130 hln 100 LABEL=1P
s 205 125 205 110 711
s 270 125 270 110 713
w 716
a 0 sr 3 0 250 179 hln 100 LABEL=101
s 270 190 270 165 715
a 0 sr 3 0 250 179 hln 100 LABEL=101
w 718
a 0 sr 3 0 159 180 hln 100 LABEL=1G
s 165 170 155 170 717
a 0 sr 3 0 159 180 hln 100 LABEL=1G
s 155 190 155 170 719
w 722
a 0 sr 3 0 158 158 hln 100 LABEL=2G
s 120 160 165 160 721
a 0 sr 3 0 158 158 hln 100 LABEL=2G
w 724
a 0 sr 3 0 207 208 hln 100 LABEL=1C
s 205 215 205 205 723
a 0 sr 3 0 207 208 hln 100 LABEL=1C
w 726
s 155 260 205 260 725
s 155 260 155 230 727
s 205 265 205 260 729
s 205 260 205 255 731
s 270 260 205 260 733
s 270 260 270 230 735
s 120 260 155 260 737
s 120 230 120 260 739
s 120 260 70 260 741
s 70 230 70 260 743
s 70 260 50 260 749
w 746
a 0 sr 3 0 122 201 hln 100 LABEL=102
s 120 200 120 220 745
a 0 sr 3 0 122 201 hln 100 LABEL=102
w 748
s 70 220 50 220 747
@junction
j 270 125
+ p 56 -
+ w 710
j 270 190
+ p 6 +
+ w 716
j 270 165
+ p 56 +
+ w 716
j 155 190
+ p 5 +
+ w 718
j 120 160
+ p 500 -
+ w 722
j 205 215
+ p 248 +
+ w 724
j 155 230
+ p 5 -
+ w 726
j 205 265
+ s 2
+ w 726
j 205 255
+ p 248 -
+ w 726
j 205 260
+ w 726
+ w 726
j 270 230
+ p 6 -
+ w 726
j 155 260
+ w 726
+ w 726
j 120 230
+ p 384 OUT-
+ w 726
j 120 260
+ w 726
+ w 726
j 70 230
+ p 384 IN-
+ w 726
j 120 220
+ p 384 OUT+
+ w 746
j 120 200
+ p 500 +
+ w 746
j 50 220
+ p 361 +
+ w 748
j 70 220
+ p 384 IN+
+ w 748
j 50 260
+ p 361 -
+ w 726
j 70 260
+ w 726
+ w 726
j 205 125
+ p 751 P
+ w 710
j 165 170
+ p 751 G
+ w 718
j 165 160
+ p 751 G2
+ w 722
j 205 205
+ p 751 C
+ w 724
@attributes
a 0 s 0 0 0 0 hln 100 PAGETITLE=
a 0 s 0 0 0 0 hln 100 PAGENO=1
a 0 s 0 0 0 0 hln 100 PAGESIZE=A
a 0 s 0 0 0 0 hln 100 PAGECOUNT=1
@text
s 5 15 95 hln 100 TRIMODE IS % MODE (0=PENTODE, 40=UL, 100=TRIODE).
