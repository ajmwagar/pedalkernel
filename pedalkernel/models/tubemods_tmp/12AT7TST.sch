*version 6.2 350650344
u 155
TU? 3
V? 3
@libraries
@analysis
.DC 1 0 0 0 1 1
+ 0 0 V_P
+ 0 4 0
+ 0 5 400
+ 0 6 4
.STEP 1 0 0
+ 0 VG
+ 4 0
+ 5 -6
+ 6 -1
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
pageloc 1 0 1685 
@status
n 0 96:08:01:14:04:00;841611840 e 
s 0 96:08:01:14:16:49;841612609 e 
*page 1 0 970 720 iA
@ports
port 2 AGND 180 225 h
@parts
part 5 VSRC 120 170 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG
a 1 xp 9 0 22 8 hcn 100 REFDES=VG
part 6 VSRC 245 165 h
a 0 x 0 0 0 0 hln 100 PKGREF=V_P
a 1 xp 9 0 -6 10 hcn 100 REFDES=V_P
part 56 VSRC 245 155 u
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VP
a 1 xp 9 0 22 38 hcn 100 REFDES=VP
part 88 12AT7 180 160 h
a 0 sp 11 0 0 10 hlb 100 PART=12AT7
a 0 a 0 0 0 0 hln 100 PKGREF=TU2
a 1 ap 9 0 -12 -6 hln 100 REFDES=TU2
part 0 titleblk 970 720 h
a 1 s 13 0 350 10 hcn 100 PageSize=A
a 1 s 13 0 180 60 hcn 100 PageTitle=
a 1 s 13 0 300 95 hrn 100 PageNO=1
a 1 s 13 0 340 95 hrn 100 PageCount=1
@conn
w 92
a 0 sr 0 0 130 156 hln 100 LABEL=1G
s 140 160 120 160 91
a 0 sr 3 0 136 156 hln 100 LABEL=1G
s 120 170 120 160 137
w 96
a 0 sr 0 0 188 117 hln 100 LABEL=1P
s 245 105 180 105 141
a 0 sr 3 0 182 115 hln 100 LABEL=1P
s 180 120 180 105 97
s 245 115 245 105 145
w 119
s 180 225 180 220 126
s 180 220 180 200 130
s 120 220 180 220 128
s 120 210 120 220 139
s 245 220 180 220 143
s 245 205 245 220 153
w 148
a 0 sr 0 0 0 0 hln 100 LABEL=101
s 245 165 245 155 151
a 0 sr 3 0 247 160 hln 100 LABEL=101
@junction
j 140 160
+ p 88 G
+ w 92
j 180 120
+ p 88 P
+ w 96
j 180 200
+ p 88 C
+ w 119
j 180 225
+ s 2
+ w 119
j 180 220
+ w 119
+ w 119
j 120 170
+ p 5 +
+ w 92
j 120 210
+ p 5 -
+ w 119
j 245 115
+ p 56 -
+ w 96
j 245 155
+ p 56 +
+ w 148
j 245 165
+ p 6 +
+ w 148
j 245 205
+ p 6 -
+ w 119
@attributes
a 0 s 0 0 0 0 hln 100 PAGETITLE=
a 0 s 0 0 0 0 hln 100 PAGENO=1
a 0 s 0 0 0 0 hln 100 PAGESIZE=A
a 0 s 0 0 0 0 hln 100 PAGECOUNT=1
@text
