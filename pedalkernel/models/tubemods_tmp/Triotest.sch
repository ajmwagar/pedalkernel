*version 6.3 931645364
lib C:\AUDIO\Tubeshar\TUBE.slb 215529251
u 225
TU? 7
V? 3
@libraries
c:\audio\tubeshar\tube  [.slb]
@analysis
.DC 1 0 0 0 1 1
+ 0 0 VP
+ 0 4 0
+ 0 5 400
+ 0 6 4
.STEP 1 0 0
+ 0 VG
+ 4 0
+ 5 -20
+ 6 -4
.OP 0 
.PROBE 0 0 1 1 0 3
.LIB c:\audio\tubeshar\tube97.lib
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
pageloc 1 0 1431 
@status
n 0 101:08:02:10:40:50;999448850 e 
s 0 101:08:02:12:21:51;999454911 e 
c 101:08:02:13:23:32;999458612
*page 1 0 970 720 iA
@ports
port 2 AGND 180 215 h
@parts
part 5 VSRC 105 165 h
a 1 u 0 0 0 0 hcn 100 DC=0
a 1 u 0 0 0 0 hcn 100 AC=0
a 0 x 0 0 0 0 hln 100 PKGREF=VG
a 1 xp 9 0 20 6 hcn 100 REFDES=VG
part 6 VSRC 235 165 h
a 0 x 0 0 0 0 hln 100 PKGREF=VP
a 1 xp 9 0 -2 6 hcn 100 REFDES=VP
part 158 12AU7 180 160 h
a 0 sp 11 0 -20 20 hlb 100 PART=12AU7
a 0 x 0 0 0 0 hln 100 PKGREF=TU1
a 0 x 0 0 0 0 hln 100 PKGTYPE=
a 1 xp 9 0 -4 8 hln 100 REFDES=TU1
part 0 titleblk 970 720 h
a 1 s 13 0 350 10 hcn 100 PageSize=A
a 1 s 13 0 180 60 hcn 100 PageTitle=
a 1 s 13 0 300 95 hrn 100 PageNO=1
a 1 s 13 0 340 95 hrn 100 PageCount=1
@conn
w 119
s 180 215 180 200 196
s 235 215 180 215 197
s 235 205 235 215 199
s 105 215 180 215 217
s 105 205 105 215 221
w 179
a 0 sr 3 0 219 134 hln 100 LABEL=1P
s 235 165 235 120 182
a 0 sr 3 0 219 130 hln 100 LABEL=1P
s 235 120 180 120 178
w 208
a 0 sr 3 0 126 156 hln 100 LABEL=1G
s 105 160 140 160 219
a 0 sr 3 0 126 156 hln 100 LABEL=1G
s 105 165 105 160 223
@junction
j 180 200
+ p 158 C
+ w 119
j 180 120
+ p 158 P
+ w 179
j 235 165
+ p 6 +
+ w 179
j 180 215
+ w 119
+ w 119
j 235 205
+ p 6 -
+ w 119
j 180 215
+ s 2
+ w 119
j 140 160
+ p 158 G
+ w 208
j 105 205
+ p 5 -
+ w 119
j 105 165
+ p 5 +
+ w 208
@attributes
a 0 s 0 0 0 0 hln 100 PAGETITLE=
a 0 s 0 0 0 0 hln 100 PAGENO=1
a 0 s 0 0 0 0 hln 100 PAGESIZE=A
a 0 s 0 0 0 0 hln 100 PAGECOUNT=1
@text
s 5 120 210 hln 80 Plate current    is -I(VP)
