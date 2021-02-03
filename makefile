Target = raspberry_upgrade_esp32


INC += -I./include
TOPDIR=..
APPDIR=src

LIBS = 


ifndef TG
	TG=PC
endif

ifeq "$(TG)" "PC"
CC = gcc
CPP = g++
else
CC = gcc -std=c99
CPP = arm-linux-gnueabihf-gcc -std=c99
endif


ifndef CFG
	CFG=Debug
endif


ifeq "$(CFG)" "Debug"
CFLAGS +=  -MMD  -w
CFLAGS += -O2
else
CFLAGS += -W
CFLAGS += -o1
endif

OUTDIR = build
COMPILE = @$(CC)  $(LIBS)  $(INC)  $(CFLAGS) -c $<   -o  $@

COMMON_OBJ=$(OUTDIR)/main.o \
	$(OUTDIR)/esp_loader.o  \
	$(OUTDIR)/example_common.o \
	$(OUTDIR)/md5_hash.o \
	$(OUTDIR)/raspberry_port.o \
	$(OUTDIR)/serial_comm.o


OBJ=$(COMMON_OBJ)
CDEF = $(OBJ:.o=.d)

all: $(Target)

$(OBJ):
# Pattern rules



$(OUTDIR)/%.o : $(APPDIR)/%.c
	@echo ""
	@echo "->>>>>>>> compile " $@
	$(COMPILE)
        

LINK =	@$(CC)  $(OBJ) $(LIBS) $(INC) -o $(Target)



# Build rules
$(Target): $(OUTDIR)  $(OBJ)
	$(LINK)
	@echo "->>>>>>>> gen " $(Target)
ifeq "$(CFG)" "Debug"
	@echo "Debug" 
else
	@echo "Release" 
endif
	@echo "+++++++++++++++++++++++++"
	@echo "+++++++++++++++++++++++++"
clean:            
	rm $(Target) $(COMMON_OBJ) $(CDEF)

