NOTE!!!

THE CONTENTS OF THIS FILE IS OBSOLETE. Kept for historical reasons only. The instructions are on

http://www.normankoren.com/Audio/Tubemodelinstr.HTM    	  	

PSPICE SHAREWARE PACKAGE
------------------------

To read this file in Notepad,
click on Edit, Word Wrap to turn on word wrap.

	For further information on these models, contact the author:

		Norman Koren	        normkoren@earthlink.net

These models are available free with no obligation as shareware unless you need professional telephone consulataion.  Actually, I enjoy casual phone conversations, as long as I have time left for my life.  I welcome feedback:  Suggestions, stories on how the models have been used, and data that could be used to extend the models.  If you come up with anything good, please consider submitting an article to Glass Audio, 305 Union St. PO Box 176, Peterborough, NH 03458-0176 USA.

PLEASE NOTE:  The parameters in my tube models are based on published data sheets, and are far from perfect for all cases.  Tubes vary from unit-to-unit and batch-to-batch.  If you measure very different curves, try experimenting with the parameters under the guidance of the first article.  I would love to see an article about which parameters vary most from tube-to-tube, and how tubes age, but I'm not holding my breath:  It would be a lot of work!

CORRECTION:  The right third was cropped out of of FIGURE 5 (in the published article).  Some filament connections are missing.  The junction of RS3 and CS5 is connected to V+320.  The hanging wire above RS3 goes through 12k resistor RS4 to V+280.  A capacitor of at least 100uF is connected between V+280 and ground.  This is a very conventional power supply with lots of shunt capacitance.

All the usual legal disclaimers apply to these models.  They were developed with the best efforts of the author, who bears no legal liability for their use or misuse.

FILES (OLD!!!)
-----

This package contains the accompanying files.

PAS94.TXT	Excerpts from the 1994 PAS modification article.  Should help
		with some of the construction details.  Turn on word wrap to view.
12AX7TST.sch	Schematic for calculating 12AX7 plate curves (used by Pspice
		Schematics program.
12AU7TST.sch	Schematic for calculating 12AU7 plate curves.
EL34PTST.sch	Schematic for calculating EL34 plate curves (pentode mode).
EL34TTST.sch	Schematic for calculating EL34 plate curves (triode mode).
EL34UTST.sch	Schematic for calculating EL34 plate curves (ultra-linear mode).
errata.txt	Errata for GA Tube models article with additional models.
Pasorgtc.cir	SPICE control file created by running Pasorgtc.sch.
Pasorgtc.net	Netlist created by running Pasorgtc.sch.
Pasorgtc.sch	Schematic for original Dynaco PAS with tone controls.
Paspursp.sch,.cir,.net  Spiced PAS (without tone controls).
Pasphon.sch	Spiced PAS phono preamp
Paspurln.sch	Spiced PAS line stage- purist mod
Pentode.cir	SPICE file for calculating pentode plate curves.
psreadme.txt	The file you are now reading!
trio300B.sch	Schematic for calculating 300B curves from Western Electric data
		reprinted in The Parts Connection Catalog & Resource Guide.
Triode.cir	SPICE file for calculating triode plate curves.
TRIOTEST.sch	Schematic for calculating generic triode plate curves.
Tube.lib	Tube model library.
Tube.slb	Tube symbol library.
Tube1.lib and Tube1.slb		Additional tube model and symbol libraries:
I have moved some devices that i'm not currently using into these files because the evaluation version of Pspice limits user-created libraries to 20 models.

The simulations in the accompanying files can be run on the MicroSim Pspice evaluation package, which consists of Schematics for entering circuits, Pspice for simulating them, Probe for viewing the simulation results, a set of documentation and tutorials, and some other programs still awaiting exploration.  This package is available free on a CD ROM from MicroSim, or can be downloaded from www.microsim.com.  I recommend ordering the CD ROM because it contains a complete set of manuals.  A list of free SPICE V3.0 packages and other useful information can be found on the World Wide Web site by searching "spice v3.0" and "spice 3.0" on your favorite search engine.  (Mine is Alta Vista.)

I am still using Pspice version 6.3 because versions 7+ limit the total number of parts more strictly (to 50).  In the newer versions, files that contain both the phono preamplifier and line amplifier (such as paspursp.sch) may need to be separated into two separate files.  For those of you who run into problems with Paspursp.sch, I have included Pasphon.sch (phono only) and Paspurln.sch (line amp only).

To order Pspice, contact

MicroSim
20 Fairbanks
Irvine, CA 92718
(800)245-3022
www.microsim.com

INTALLING THE TUBE AND SYMBOL LIBRARIES
---------------------------------------

For MicroSim Schematics to recognize the new symbol library (pathname\TUBE.SLB), click on Options, Editor Configuration..., Library Settings.  Type the full name of the new symbol library file in the Library Name box, then click Add*, OK, OK.  For Schematics to recognize a new model library (pathname\TUBE.LIB), click on Analysis, Library and Include Files...  Type the full name of the new model library file in the File Name box, then click Add Library*, OK.

GETTING STARTED WITH PSPICE
---------------------------

The following is an excerpt from a future article (more or less):

Although running Pspice from ASCII data files is the fastest way to get started, entering schematic diagrams via the Pspice Schematics program is the most satisfying and accurate way to operate in the long run.  Since MicroSim does not supply a library of tube symbols, you will need to create your own*.  [*This will appear in print in a future article.  You don't need to do so if you install accompanying files TUBE.LIB and TUBE.SLB!]  Libraries of schematic symbols are stored in files with the extension .SLB and libraries of SPICE models are stored in files with the extension .LIB.  (Each of these files contains symbols or models for several parts: up to 25 in the MicroSim evaluation package.)  I use TUBE.SLB and TUBE.LIB for my tube drawings and models.

The technique for linking drawings to models is quite complex, and the MicroSim tutorial must be studied to grasp it.  It would be wise to look it over before continuing with this paragraph!  To call it from Schematics, click on Help, Contents, and then click on the topic of your choice.  The following steps will bring up a good example that can be adapted for tube models.  From Schematics, click on File, Edit Library to get into the symbol library editor.  Click on File, Open..., eval.slb to open a library file that contains some parts useful as examples.  (Be patient:  Your screen will still be blank at this point.)  Click on Part, Get..., IRF9410 to load the drawing for a P-channel mosfet.  (Choose another part if you desire, but this MOSFET is as close as you can get to a tube in this library.)  Click on Part, Attributes... to see the statements that define this part.  For the IRF9410, they are

template=M^@refdes %d %g %s %s @MODEL
refdes=M?
PART=IRF9140
MODEL=IRF9140

Template links the part to the model library:  With appropriate substitutions, for example, IRF9140 for @MODEL, it appears in the netlist.  A number on the schematic diagram replaces ? in refdes.  To alter an attribute, click on it in the box on the right and edit it in the boxes on the upper left.  The attributes of the 12AX7 that link it to its subcircuit in TUBE.LIB are:

template=X^@REFDES %P %G %C @MODEL
refdes=TU?
PART=12AX7
MODEL=12AX7

A part may be copied for modification by clicking on Part, Copy, and then entering the appropriate old and new names.  To create a new library file (e.g., TUBE.SLB) to save your new parts, click on File, New, then File, Save as..., TUBE.SLB (or a full path name of your choice).  To insert a part into the new library file, draw it following instructions in the tutorial or modify an existing part (such as the IRF9410), then save it by clicking Part, Save to Library...  Either type the library file name or use the usual Windows navigation techniques to locate it.  Click on File, Close to return to the Schematics editor.  The tutorial contains much more information on how to create and modify symbols for new parts..

For MicroSim Schematics to recognize a new symbol library (the file pathname\TUBE.SLB that you just created), click on Options, Editor Configuration..., Library Settings.  Type the full name of the new symbol library file in the Library Name box, then click Add*, OK, OK.  For Schematics to recognize a new model library (the file pathname\TUBE.LIB), click on Analysis, Library and Include Files...  Type the full name of the new model library file in the File Name box, then click Add Library*, OK.

Parts are entered into MicroSim Schematics drawings by clicking on the little box on the toolbar with the yellow glove and selecting them or by entering a part name such "R", "L", "C", "BUBBLE", etc. in the box to the right of the glove.  I recommend using the rubberband option, which can be set by clicking Options, Display Options..., Rubberband.   Named nodes (1G, 1P, etc.), which are created by double-clicking on lines and entering a name, are required for voltages to be examined in Probe.  Input files to Schematics have the three character extension .SCH.  Clicking Analysis, Create Netlist creates files with extensions .CIR (the control file), .NET (the netlist), and .ALS (an alias file which speeds up analysis) which are used as inputs to Pspice.  The .CIR and .NET files contain ASCII text in SPICE format, as described in Paul Tuinenga's excellent text10.  Clicking Analysis, Simulate creates the netlist, runs Pspice, and loads Probe to display results.  Press the Insert key to select a voltage or current for Probe display.  You can use "VDB" or "VP" to display a voltage in dB or its phase in degrees.  There are many more possibilities for controlling the display.

Files 12AX7TST.SCH, EL34PTST.SCH, and the others with similar names illustrate Schematics circuits for triodes and pentodes that can be used to generate characteristic curves.  Their netlists are similar to files Triode.cir and Pentode.cir.  Each of these circuits requires that you perform a few added steps to generate the characteristic curves.  We will use the pentode as an example.

To step the grid voltage, click on Analysis, Setup..., Parametric...  Set Swept Var. Type to Voltage Source, Sweep Type to Linear, Name: to VG1, Start Value to 0, End Value: to -75 (or as appropriate), and Increment: to -15 (or as appropriate), then click OK.  You may set the plate voltage sweep while the Analysis Setup... box is still open.  Click DC Sweep...  Set Swept Var. Type to Voltage Source, Sweep Type to Linear, Name: to V_P, Start Value to 0, End Value: to 600 (or as appropriate), and Increment: to 5 (or as appropriate), then click OK.  The DC Sweep... and Parametric boxes should contain check marks.  Finally, click Close.  All voltage sources except VG2NOM are set to DC=0 in the schematic diagram.  (Attributes can be viewed and changed by double-clicking on the symbol.)  Grid and plate voltages are controlled by the DC Sweep... and Parametric... settings.

VG2NOM is the nominal screen grid voltage.  It has been set to 300V in Figure 3 (pentode mode) and 450V in Figure 5 (ultra-linear (UL) mode).  (It has no meaning in triode mode, where screen grid voltage equals the plate voltage).  E2 is an EVAL voltage-controlled voltage source that controls how the screen grid voltage tracks the plate voltage.  The value shown in Figure 8b, V(%IN+, %IN-)*.4+V(101)*.6, is for ultra- linear mode.  For pentode mode, set it to V(%IN+, %IN-).  For triode mode, set it to V(101).  VP and VC are both set to zero:  They have been entered as shown so that I(VP) and I(VC) have the correct polarity when plotted by Probe.  When you have completed entering the schematic, click Analysis, Simulate.  This will create a netlist, run Pspice, and then run Probe to display the results.  Error messages can appear at any one of several stages of this process.  They are unavoidable when you start out.  Don't get frustrated.  This is where real learning happens.
