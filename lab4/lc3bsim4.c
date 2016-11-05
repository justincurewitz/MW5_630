/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();
void copyIntAr(int* src, int* dup, int len);

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
/* MODIFY: you have to add all your new control signals */
		LD_SS,
		LD_US,
		STMUX,
		GateSTMUX,
		ZEXTMUX,
		INVERT,
		SR2MUX,
		GatePSR,
		VMUX,
		GateVMUX,
		LD_PSR,
		SR6MUX,
		AINT,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }
/* MODIFY: you can add more Get functions for your new control signals */
int	GetLD_SS(int *x)				 { return(x[LD_SS]);}
int	GetLD_US(int *x)			   { return(x[LD_US]);}
int	GetSTMUX(int *x)				 { return(x[STMUX]);}
int	GetGateSTMUX(int *x)		 { return(x[GateSTMUX]);}
int	GetZEXTMUX(int *x)			 { return(x[ZEXTMUX]);}
int	GetINVERT(int *x)				 { return(x[INVERT]);}
int	GetSR2MUX(int *x)				 { return(x[SR2MUX]);}
int	GetGatePSR(int *x)			 { return(x[GatePSR]);}
int	GetVMUX(int *x)					 { return(x[VMUX]);}
int	GetGateVMUX(int *x)			 { return(x[GateVMUX]);}
int	GetLD_PSR(int *x)				 { return(x[LD_PSR]);}
int	GetSR6MUX(int *x)				 { return(x[SR6MUX]);}
int	GetAINT(int *x)					 { return(x[AINT]);}

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 

/* For lab 4 */
int INT; /*Interrupt triggered*/
int EXC; /*Exception triggered*/
int INTV; /* Interrupt vector register */
int EXCV; /* Exception vector register */
int SSP; /* Initial value of system stack pointer */
/* MODIFY: You may add system latches that are required by your implementation */
int USP; /*User Stack Pointer*/
int PSR; /*Program status register*/
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%04x..0x%04x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%04x (%d) : 0x%02x%02x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%04x..0x%04x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%04x (%d) : 0x%02x%02x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%04x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%04x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%04x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%04x\n", BUS);
    printf("MDR          : 0x%04x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%04x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%04x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%04x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%04x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%04x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%04x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%04x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%04x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%04x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : init_memory                                     */
/*                                                             */
/* Purpose   : Zero out the memory array                       */
/*                                                             */
/***************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files) { 
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(program_filename);
	while(*program_filename++ != '\0');
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], argv[2], argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code, except for the places indicated 
   with a "MODIFY:" comment.

   Do not modify the rdump and mdump functions.

   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

void eval_micro_sequencer() {
  /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */
	if (CYCLE_COUNT == 0) {CURRENT_LATCHES.PSR = 0x8000;}
	int exception = checkException();
  if (GetIRD(CURRENT_LATCHES.MICROINSTRUCTION)) {
    int next_micro = (CURRENT_LATCHES.IR & 0x0000F000) >> 12;
    copyIntAr(CONTROL_STORE[next_micro], (int*)NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE_BITS);
    NEXT_LATCHES.STATE_NUMBER = next_micro;
  } else {
		if (exception) {
			copyIntAr(CONTROL_STORE[63], (int*)CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE_BITS);
			CURRENT_LATCHES.EXCV = exception;
		}
    int j = GetJ(CURRENT_LATCHES.MICROINSTRUCTION);
    int cond_bits = GetCOND(CURRENT_LATCHES.MICROINSTRUCTION);
    int cond1 = (cond_bits&0x2) >> 1;
    int cond0 = cond_bits & 0x1;
    int ir11 = (CURRENT_LATCHES.IR&0x0800) >> 11;
    cond_bits = 0;
    cond_bits = ((cond1&(~cond0)&CURRENT_LATCHES.BEN) << 2) + (((~cond1)&cond0&CURRENT_LATCHES.READY) << 1) + (cond1&cond0&ir11);
    j |= cond_bits;
		if (GetAINT(CURRENT_LATCHES.MICROINSTRUCTION)) {
			j |= CURRENT_LATCHES.INT << 5;
		}
    copyIntAr(CONTROL_STORE[j], (int*)NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE_BITS);
    NEXT_LATCHES.STATE_NUMBER = j;
  }
}

int mem_cycle = 0;
void cycle_memory() {
 
  /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */
   if (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION)) {
     mem_cycle += 1;
     if (mem_cycle == 4) {
       NEXT_LATCHES.READY = 1;
       mem_cycle = 0;
     } else {
       NEXT_LATCHES.READY = 0;
     }
   } else {
     mem_cycle = 0;
     NEXT_LATCHES.READY = 0;
   }

}



int newMARMUX;
int PC;
int ALU;
int SHF;
int MDR;
int evalMARMUX(void);
int evalPC(void);
int evalALU(void);
int evalSHF(void);
int evalMDR(void);
int evalAddressAdder(void);
int getDR(void);
int getBaseR(void);
int getSR1(void);
int getSR2(void);
int getBEN(void);
int isEqual(int* ar1, int* ar2, int len);
int checkException(void);

void eval_bus_drivers() {

  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */    
  newMARMUX = evalMARMUX();
  PC = evalPC();
  ALU = evalALU();
  SHF = evalSHF();
  MDR = evalMDR();
}


/*solves for MARMUX */
int evalMARMUX(void) {
  if (GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 0) {
    int val = (CURRENT_LATCHES.IR&0xFF) << 1;
		if (GetZEXTMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {
			val = val + 0x0200;
		}
    return val;
  } else {
    return evalAddressAdder();
  }
}

/*solves for PC */
int evalPC(void) {
  int pcmux = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
  if (GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION)) {
    if (pcmux == 0) {return CURRENT_LATCHES.PC + 2;}
    else if (pcmux == 1) {return BUS;} 
    else if (pcmux == 2) {return evalAddressAdder();}
    else if (pcmux == 3) {return CURRENT_LATCHES.PC -2;}
  }
}

/*solves for ALU */
int evalALU(void){
  int cmd = GetALUK(CURRENT_LATCHES.MICROINSTRUCTION);
  int sr1 = CURRENT_LATCHES.REGS[getSR1()];
  int sr2 = getSR2();
  if (cmd == 0) {return Low16bits(sr1 + sr2);}
  else if (cmd == 1) {return Low16bits(sr1 & sr2);}
  else if (cmd == 2) {return Low16bits(sr1 ^ sr2);}
  else return Low16bits(sr1);
}

/* solves for the value at shf */
int evalSHF(void){
  int sr1 = CURRENT_LATCHES.REGS[getSR1()];
  int shifts = CURRENT_LATCHES.IR & 0x000F;
  int type = (CURRENT_LATCHES.IR & 0x0030) >> 4;
  if (type == 0) {return Low16bits(sr1 << shifts);}
  else if (type == 1) {return Low16bits(sr1 >> shifts);}
  else if (type == 3) {
    if (sr1&0x00008000) {sr1 |= 0xFFFF0000;}
    sr1 = Low16bits(sr1 >> shifts);
    return sr1;
  }
}

/* solves for the value in MDR */
int evalMDR(void){
  int mio = GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION);
  int rw = GetR_W(CURRENT_LATCHES.MICROINSTRUCTION);
  if (GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION)) {
    if (mio == 0) {
      int ds = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
      if (ds == 0) {
        if (CURRENT_LATCHES.MAR&0x1) {
          return Low16bits(BUS);
        } else {
          return Low16bits(BUS);
        }
      } else {
        return Low16bits(BUS);
      }
    } else {
      return MEMORY[CURRENT_LATCHES.MAR>>1][0] + (MEMORY[CURRENT_LATCHES.MAR>>1][1] << 8);
    }
  } else {
    return MDR;
  }
}

/* gets the value of the Address Adder */
int evalAddressAdder(void) {
  int right;
  int left;
  if (GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
    right = CURRENT_LATCHES.REGS[getSR1()];
  } else {
    right = CURRENT_LATCHES.PC;
  }
  int ADR2mux = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
  if (ADR2mux == 0) {left = 0;}
  else if (ADR2mux == 1) {
    left = CURRENT_LATCHES.IR&0x3F;
    if (left&0x20) {left |= 0xFFC0;}
  }
  else if (ADR2mux == 2) {
    left = CURRENT_LATCHES.IR&0x1FF;
    if (left&0x0100) {left |= 0xFE00;}
  }
  else if (ADR2mux == 3) {
    left = CURRENT_LATCHES.IR&0x7FF;
    if (left&0x0400) {left |= 0xF800;}
  }
  if (GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION)) {left = left << 1;}
  return Low16bits(left + right);
}

/*gets the DR value */
int getDR(void) {
	int DR2 = GetSR6MUX(CURRENT_LATCHES.MICROINSTRUCTION) | GetGateSTMUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (DR2 == 0) {
		if (GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {return 7;}
		else {return (CURRENT_LATCHES.IR&0x0E00) >> 9;}
	} else {
		if (GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {return 6;}
		else {return 0;}
	}
}

/*checks to see if two arrays are equivalent */
int isEqual(int* ar1, int* ar2, int len) {
  int i;
  for (i = 0; i < len; ++i) {
    if (ar1[i] != ar2[i]) {return 0;}
  }
  return 1;
}

/*gets the BaseR value from the IR */
int getBaseR(void) {
  int BaseR = (CURRENT_LATCHES.IR&0x01C0) >> 6;
  return BaseR;
}

/* gets the value in SR1 */
int getSR1(void) {
  int sr1;
	int sr6 = GetSR6MUX(CURRENT_LATCHES.MICROINSTRUCTION);
  int srmux = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (sr6 == 0) {
		if (srmux == 0) {sr1 = (CURRENT_LATCHES.IR&0x0E00) >> 9;}
		else {sr1 = (CURRENT_LATCHES.IR&0x01C0) >> 6;}
	} else {
		if (srmux == 0) {sr1 = 0;}
		else {sr1 = 6;}
	}	
  return sr1;
}

/* gets the SR2 bits */
int getSR2(void) {
	int sr2m = GetSR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (sr2m) {
		if (GetINVERT(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {return -2;}
		else {return 2;}
	}
  if (CURRENT_LATCHES.IR&0x0020) {
    int sr2 = CURRENT_LATCHES.IR&0x001F;
    if (sr2&0x10){sr2 |= 0xFFE0;}
    return Low16bits(sr2);
  }
  else {
    return CURRENT_LATCHES.REGS[CURRENT_LATCHES.IR&0x0007];
  }
}

void drive_bus() {

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */       
  if (GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION)) {BUS = CURRENT_LATCHES.PC;}
  else if (GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION)) {
    int datasize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
    if (datasize == 0) {
      if (CURRENT_LATCHES.MAR%2){
				int temp = MDR >> 8;
				if (temp&0x0080) {temp |= 0xFF00;}
        BUS = temp;
      }
      else {
				int temp = MDR&0xFF;
				if (temp&0x0080) {temp |= 0xFF00;}
				BUS = temp;
      }
    } else {
      BUS = MDR;
    }
  }
  else if (GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION)) {
    BUS = ALU;
  }
  else if (GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
    BUS = newMARMUX;
  }
  else if (GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION)) {
    BUS = SHF;
  }
	else if (GetGateSTMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
		if (GetSTMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {BUS = CURRENT_LATCHES.USP;}
		else {BUS = CURRENT_LATCHES.SSP;}
	}
	else if (GetGateVMUX(CURRENT_LATCHES.MICROINSTRUCTION)) {
		if (GetVMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {
      BUS = CURRENT_LATCHES.EXCV;
    }
		else {
			BUS = CURRENT_LATCHES.INTV;
			CURRENT_LATCHES.INT = 0;
		}
	}
	else if (GetGatePSR(CURRENT_LATCHES.MICROINSTRUCTION)) {
		BUS = CURRENT_LATCHES.PSR;
	}
  else {BUS = 0;}
}

void latch_datapath_values() {

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */       
  if (GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.PC = evalPC();}
  if (GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.MDR = evalMDR();}
  if (GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.MAR = BUS;}
  if (GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.IR = BUS;}
  if (GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.REGS[getDR()] = BUS;}
  if (GetLD_BEN(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.BEN = getBEN();}
  if (GetLD_CC(CURRENT_LATCHES.MICROINSTRUCTION)) {
    if (BUS&0x8000) {
      NEXT_LATCHES.N = 1;
      NEXT_LATCHES.Z = 0;
      NEXT_LATCHES.P = 0;
    } else if (BUS == 0) { 
      NEXT_LATCHES.N = 0;
      NEXT_LATCHES.Z = 1;
      NEXT_LATCHES.P = 0;
    } else if (BUS > 0) {
      NEXT_LATCHES.N = 0;
      NEXT_LATCHES.Z = 0;
      NEXT_LATCHES.P = 1;
    }
		int cc = (NEXT_LATCHES.N << 2) | (NEXT_LATCHES.Z << 1) | (NEXT_LATCHES.P);
		CURRENT_LATCHES.PSR &= 0xFFF8;
		CURRENT_LATCHES.PSR |= cc;
  }
	if (GetLD_SS(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.SSP = BUS;}
	if (GetLD_US(CURRENT_LATCHES.MICROINSTRUCTION)) {NEXT_LATCHES.USP = BUS;}
	if (GetLD_PSR(CURRENT_LATCHES.MICROINSTRUCTION)) {
		if (GetGatePSR(CURRENT_LATCHES.MICROINSTRUCTION)) {
			NEXT_LATCHES.PSR = CURRENT_LATCHES.PSR&0x7FFF;
		} else {
			NEXT_LATCHES.PSR = BUS;
		}
	} else {NEXT_LATCHES.PSR = CURRENT_LATCHES.PSR;}
  int datasize = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
  if (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) && GetR_W(CURRENT_LATCHES.MICROINSTRUCTION)) {
    if (datasize == 1) {
      MEMORY[CURRENT_LATCHES.MAR>>1][0] = (CURRENT_LATCHES.MDR&0xFF);
      MEMORY[CURRENT_LATCHES.MAR>>1][1] = (CURRENT_LATCHES.MDR >> 8)&0xFF;
    } else {
      int byte = CURRENT_LATCHES.MAR&0x1;
      MEMORY[CURRENT_LATCHES.MAR>>1][byte] = (CURRENT_LATCHES.MDR &0xFF);
    }
  }
	if (CYCLE_COUNT == 299) {
		NEXT_LATCHES.INT = 1;
		NEXT_LATCHES.INTV = 0x0001;
	} else {
		NEXT_LATCHES.INT = CURRENT_LATCHES.INT;
		NEXT_LATCHES.INTV = CURRENT_LATCHES.INTV;
	}
  NEXT_LATCHES.EXCV = CURRENT_LATCHES.EXCV;
}

/*solves for BEN*/
int getBEN(void) {
  int n = ((CURRENT_LATCHES.IR&0x0800) >> 11) & CURRENT_LATCHES.N;
  int z = ((CURRENT_LATCHES.IR&0x0400) >> 10) & CURRENT_LATCHES.Z;
  int p = ((CURRENT_LATCHES.IR&0x0200) >> 9) & CURRENT_LATCHES.P;
  return n | z | p;
}

/*returns copy of src */
int*  intdup(int* src, int len) {
  int i;
  int * dup = calloc(len, sizeof(int));
  for(i = 0; i < len; ++i) {
    dup[i] = src[i];
  }
  return dup;
}

/*copies array src into dup */
void copyIntAr(int* src, int* dup, int len) {
  int i;
  for(i = 0; i < len; ++i) {
    dup[i] = src[i];
  }
}

int checkException(void) {
  if (CURRENT_LATCHES.PSR&0x8000 == 0 && CURRENT_LATCHES.MAR < 0x3000 && GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) == 1) {
    NEXT_LATCHES.EXCV = 2;
		NEXT_LATCHES.STATE_NUMBER = 0x3F;
    printf("Protection Exception!\n");
    return 2;
  } else if (GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION) == 1 && GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1 && CURRENT_LATCHES.MAR&0x1 == 1) {
    NEXT_LATCHES.EXCV = 3;
		NEXT_LATCHES.STATE_NUMBER = 0x3F;
    printf("Unaligned Access Exception!\n");
    return 3;
  } else if (CURRENT_LATCHES.STATE_NUMBER == 0xA || CURRENT_LATCHES.STATE_NUMBER == 0xB) {
    NEXT_LATCHES.EXCV = 4;
		NEXT_LATCHES.STATE_NUMBER = 0x3F;
    printf("Unknown Opcode Exception!\n");
    return 4;
  }
  else {return 0;}
}
