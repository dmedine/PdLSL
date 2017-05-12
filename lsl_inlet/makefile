NAME = lsl_inlet
CSYM = lsl_inlet

# this is the UNIX-style complicated layout dir, simple goes to $(prefix)/pd
#prefix = /usr/local
#libpddir = $(prefix)/lib/pd

.PHONY: 

current: pd_nt


# ----------------------- Microsoft Visual C -----------------------
MSCC = cl
MSLN = link

pd_nt: $(NAME).dll

.SUFFIXES: .dll

PDNTCFLAGS = -W3 -WX -DNT -DPD -nologo -D_CRT_SECURE_NO_WARNINGS \
    -D_CRT_NONSTDC_NO_DEPRECATE
VC = "C:\\Program Files (x86)\\Microsoft Visual Studio 9.0\\VC"
VSTK = "C:\\Program Files\\Microsoft SDKs\\Windows\\v6.0A"
PDPATH = "C:\\Users\\David.Medine\\Pd"

PDNTINCLUDE = -I. -I$(PDPATH)\\src -I$(VC)\\include -I$(VSTK)\\include

PDNTLDIR = $(VC)\\lib
PDNTLIB = -NODEFAULTLIB:libcmt -NODEFAULTLIB:oldnames -NODEFAULTLIB:kernel32 \
        -NODEFAULTLIB:uuid \
	$(PDNTLDIR)\\libcmt.lib $(PDNTLDIR)\\oldnames.lib \
        $(VSTK)\\lib\\kernel32.lib $(VSTK)\\lib\\uuid.lib \
	$(PDPATH)\\bin\\pd.lib .\\liblsl32.lib

.c.dll:
	$(MSCC) $(PDNTCFLAGS) $(PDNTINCLUDE) -c $*.c
	$(MSLN) -nologo -dll -export:$(CSYM)_setup $*.obj $(PDNTLIB)

# ----------------------- LINUX i386 -----------------------

pd_linux: $(NAME).pd_linux

.SUFFIXES: .pd_linux

LINUXCFLAGS = -DPD -O2 -funroll-loops -fomit-frame-pointer -fPIC \
    -Wall -W -Wshadow -Wstrict-prototypes \
    -Wno-unused -Wno-unused-parameter -Wno-parentheses -Wno-switch \
    $(CFLAGS) $(MORECFLAGS)

LINUXINCLUDE =  -I../../src

.c.pd_linux:
	$(CC) $(LINUXCFLAGS) $(LINUXINCLUDE) -o $*.o -c $*.c
	$(CC) -shared -o $*.pd_linux $*.o -lc -lm
	rm -f $*.o
