;TTL Interrupt-Serial I/O Driver
;****************************************************************
;Initializes and contains subroutines for Interrupt based
;serial IO
;Name:  Zach Morgan
;Date:
;Class:  CMPE-250
;Section:  Section 5, 5:30pm-7:30pm Wed.
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;September 25, 2017
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  	EQU  	(1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2  EQU		(SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R  EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0
;---------------------------------------------------------------
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
UART0_C2_T_RI  EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R) 
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI) 
	
	
	
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
			EXPORT	PutChar
            
Reset_Handler  PROC  {},{}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
			BL		Init_UART0_IRQ
			CPSIE	I
			
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			LDR     R0,=QBuffer     ;Load inputs for Queue Initialization
            LDR     R1,=QRecord     ;R0 = Buffer pointer, R1 = Record Pointer, R2 = Buffer Size
            MOVS    R2,#Q_BUF_SZ
            BL      InitQueue       ;Initialize an Empty Queue with a size of 4 Bytes
            ;Initialize Command Characters
MainLoop    MOVS    R2,#0x44    ;R2 <- 'D'
            MOVS    R3,#0x45    ;R3 <- 'E'
            MOVS    R4,#0x48    ;R4 <- 'H'
            MOVS    R5,#0x50    ;R5 <- 'P'
            MOVS    R6,#0x53    ;R6 <- 'S'
            ;Clears the C Flag incase it is still set from the loop
            MRS   	R0,APSR
			MOVS  	R7,#0x20
			LSLS  	R7,R7,#24
			BICS  	R0,R0,R7
			MSR   	APSR,R0
            ;Prints the prompt and prepares for user input
			LDR		R0,=Test
			BL		PutString
            LDR     R0,=Prompt
            BL      PutString
            MOVS    R0,#0
            BL      GetChar
            
            ;Check for Upper or Lower case 'D'
            CMP     R0,R2
            BEQ     DequeueB
            ADDS    R2,R2,#0x20
            CMP     R0,R2
            BEQ     DequeueB
            ;Check for Upper or Lower case 'E'
            CMP     R0,R3
            BEQ     EnqueueB
            ADDS    R3,R3,#0x20
            CMP     R0,R3
            BEQ     EnqueueB
           ;Check for Upper or Lower case 'H'
            CMP     R0,R4
            BEQ     Help
            ADDS    R4,R4,#0x20
            CMP     R0,R4
            BEQ     Help
            ;Check for Upper or Lower case 'P'
            CMP     R0,R5
            BEQ     Print
            ADDS    R5,R5,#0x20
            CMP     R0,R5
            BEQ     Print
            ;Check for Upper or Lower case 'S'
            CMP     R0,R6
            BEQ     StatusB
            ADDS    R6,R6,#0x20
            CMP     R0,R6
            BEQ     StatusB
            
            ;If the mainloop falls through to this point the input is not recognized
            LDR     R0,=InvalidCommand
            BL      PutString
            BL      NewLine
            B       MainLoop

            ;Point where program branches to if a 'd' or 'D' is input
DequeueB    BL      PutChar         ;Prints the input first
            BL      NewLine         ;Prints a new line 
            LDR     R1,=QRecord     ;R1 should already contain the record but reloads just incase
            BL      Dequeue         ;Dequeues and branches to the failure label if the C flag is set
            BCS     Failure
            BL      PutChar         ;Prints the dequeued char
            MOVS    R0,#0x3A
            BL      PutChar
            B       Status
      
           ;Point where program branches to if a 'e' or 'E' is input
EnqueueB    BL      PutChar         ;Prints the input first
            BL      NewLine         ;Prints a new line 
            LDR     R0,=EnQ         ;Loads and prints the 'Character to Enqueue' Prompt
            BL      PutString
            BL      GetChar         ;Gets and prints the char to be enqueued
            BL      PutChar
            LDR     R1,=QRecord     ;Reloads qRegister just in case
            BL      Enqueue         ;Enqueues the input character and branches to the failure lable if the C Flag is set
            BL      NewLine
            BCS     Failure
            LDR     R0,=SuccessPrompt
            BL      PutString
            B       Status
            
            ;Point where program branches to if a 'h' or 'H' is input
Help        BL      PutChar         ;Prints the input first
            BL      NewLine         ;Prints a new line 
            LDR     R0,=HelpPrompt  ;Loads and prints the help prompt
            BL      PutString
            BL      NewLine
            B       MainLoop
            
            ;Point where program branches to if a 'p' or 'P' is input
Print       BL      PutChar         ;Prints the input first
            BL      NewLine         ;Prints a new line 
            BL      PrintQueue      ;Prints the contents of the queue
            BL      NewLine         ;Prints a new line 
            B       MainLoop
            
StatusB     BL      PutChar     ;Point where program branches to if a 's' or 'S' is input
Status      BL      NewLine     ;Generic status used for status input and after Enqueue and Dequeue operation
            LDR     R0,=In      ;R0 <- "In=0x"
            BL      PutString
            LDR     R1,=QRecord ;R1 <- QRecord
            LDR     R0,[R1,#IN_PTR] ;R0 <- Address of In Pointer
            BL      PutNumHex
            LDR     R0,=Out     ;R0 <- "Out=0x"
            BL      PutString
            LDR     R0,[R1,#OUT_PTR] ;R0 <- Address of Out Pointer
            BL      PutNumHex
            LDR     R0,=Num     ;R0 <- "Num="
            BL      PutString
            LDRB    R0,[R1,#NUM_ENQD] ;R0 <- Number of Characters in Queue
            BL      PutNumUB
            BL      NewLine
            B       MainLoop

            ;Either Dequeue or Enqueue branch here if the operation was unsuccessful
Failure     LDR     R0,=FailurePrompt
            BL      PutString
            B       Status
;>>>>>   end main program code <<<<<

;>>>>>	 begin subroutine code <<<<<
;Title: UART Initialization for interrupt based serial I/O 
;Functionality: This subroutine prepares the board for UART input and output
;With the format of a Baud rate of 9600, 8 data bits, no parity
;and 1 stop bit.
;Input: No input required
;Ouput: No output
;Register Modification List: NONE

Init_UART0_IRQ  		PROC		{R0-R14}
			PUSH 		{R0-R3,LR}

;Initializes the Rx and Tx queues
			LDR			R0,=RxBuffer
			LDR			R1,=RxRecord
			MOVS		R2,#0x50
			BL			InitQueue
			LDR			R0,=TxBuffer
			LDR			R1,=TxRecord
			BL			InitQueue
			
;Select MCGPLLCLK / 2 as UART0 clock source
			LDR   		R0,=SIM_SOPT2
			LDR   		R1,=SIM_SOPT2_UART0SRC_MASK
			LDR   		R2,[R0,#0]
			BICS 		R2,R2,R1
			LDR   		R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS  		R2,R2,R1
			STR   		R2,[R0,#0]

;Enable external connection for UART0
			LDR   		R0,=SIM_SOPT5
			LDR   		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR   		R2,[R0,#0]
			BICS  		R2,R2,R1
			STR   		R2,[R0,#0]

;Enable clock for UART0 module
			LDR   		R0,=SIM_SCGC4
			LDR   		R1,=SIM_SCGC4_UART0_MASK
			LDR   		R2,[R0,#0]
			ORRS  		R2,R2,R1
			STR   		R2,[R0,#0]

;Enable clock for Port A module
			LDR   		R0,=SIM_SCGC5
			LDR   		R1,=SIM_SCGC5_PORTA_MASK
			LDR   		R2,[R0,#0]
			ORRS  		R2,R2,R1
			STR   		R2,[R0,#0]

;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR     	R0,=PORTA_PCR1
			LDR     	R1,=PORT_PCR_SET_PTA1_UART0_RX
			STR     	R1,[R0,#0]

;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR     	R0,=PORTA_PCR2
			LDR     	R1,=PORT_PCR_SET_PTA2_UART0_TX
			STR     	R1,[R0,#0]

;Load base address for UART0
			LDR			R0,=UART0_BASE

;Disable UART0
			MOVS   		R2,#UART0_C2_T_R
			LDRB   		R1,[R0,#UART0_C2_OFFSET]
			BICS   		R1,R1,R2
			STRB   		R1,[R0,#UART0_C2_OFFSET]


;Set UART0 IRQ priority    
			LDR     	R0,=UART0_IPR    
			;LDR     	R1,=NVIC_IPR_UART0_MASK    
			LDR     	R2,=NVIC_IPR_UART0_PRI_3    
			LDR     	R3,[R0,#0]    
			;BICS    	R3,R3,R1    
			ORRS    	R3,R3,R2    
			STR     	R3,[R0,#0] 
			
;Clear any pending UART0 interrupts    
			LDR     	R0,=NVIC_ICPR    
			LDR     	R1,=NVIC_ICPR_UART0_MASK    
			STR     	R1,[R0,#0] 
			
;Unmask UART0 interrupts    
			LDR     	R0,=NVIC_ISER    
			LDR     	R1,=NVIC_ISER_UART0_MASK
			STR			R1,[R0,#0]


;Set UART0 baud rate—BDH before BDL
			MOVS  		R2,#UART0_BDH_9600
			STRB  		R2,[R0,#UART0_BDH_OFFSET]
			MOVS  		R2,#UART0_BDL_9600
			STRB 		R2,[R0,#UART0_BDL_OFFSET]

;Set UART0 character format for serial bit stream and clear flags
			MOVS  		R2,#UART0_C1_8N1
			STRB  		R2,[R0,#UART0_C1_OFFSET]
			MOVS  		R2,#UART0_C3_NO_TXINV
			STRB 		R2,[R0,#UART0_C3_OFFSET]
			MOVS  		R2,#UART0_C4_NO_MATCH_OSR_16
			STRB  		R2,[R0,#UART0_C4_OFFSET]
			MOVS  		R2,#UART0_C5_NO_DMA_SSR_SYNC
			STRB  		R2,[R0,#UART0_C5_OFFSET]
			MOVS  		R2,#UART0_S1_CLEAR_FLAGS
			STRB  		R2,[R0,#UART0_S1_OFFSET]
			MOVS  		R2, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB  		R2,[R0,#UART0_S2_OFFSET]
			
;Enable UART0 transmitter, receiver, and receive interrupt
			MOVS  		R2,#UART0_C2_T_RI
			STRB  		R2,[R0,#UART0_C2_OFFSET]

			

			POP	   		{R0-R3,PC}
			BX	    	LR
			ENDP


;Title: UART0 Interrupt Service Routine
;Description: Interrupt Service Routine that handles UART0
;trasmit and receive interrupts
;Input: none

UART0_ISR
			PUSH	{R4-R7}
			
			CPSID		I							;Mask Interrupts
			LDR     	R5,=UART0_BASE				;Load UART base -> R5, stays there for entire ISR
			MOVS		R1,#UART0_C2_TI_RI			
			LDRB		R2,[R5,#UART0_C2_OFFSET]
			ANDS		R2,R2,R1					;Check if TxInterruptEnabled
			BEQ			Rx							;If it isn't go to RxInterrupt check
Wait		MOVS		R2,#UART0_S1_TDRE_MASK
			LDRB		R1,[R5,#UART0_S1_OFFSET]
			ANDS		R1,R1,R2					;checks if TxInterrupt is present
			BEQ			Rx						;loops in place until the interrupt
			LDR			R1,=TxRecord					
			BL			Dequeue						;Dequeues from the transmit queue when ready
			BCS			DisableTx
			;MOVS		R3,#UART0_D_OFFSET					;If it is unsucessful the Trasmit interrupt is disabled
			;LDR			R2,=(UART0_BASE + UART0_D_OFFSET)
			
			STRB		R0,[R5,#UART0_D_OFFSET]		;if it is sucessful the dequeued character is stored to the UART Data register
			B			EndISR
			
			;Disables the Trasmit interrupt
DisableTx	MOVS		R0,#UART0_C2_T_RI
			STRB		R0,[R5,#UART0_C2_OFFSET]
			B			EndISR
			
Rx			MOVS		R1,#UART0_S1_RDRF_MASK
			LDRB		R2,[R5,#UART0_S1_OFFSET]
			ANDS		R2,R2,R1
			BEQ			EndISR							;Loops until a character is received by the UART
			LDRB		R0,[R5,#UART0_D_OFFSET]		;Loads the newly trasmitted character 
			LDR			R1,=RxRecord
			BL			Enqueue						;Places the received character in the receive queue
			


EndISR		POP			{R4-R7}
			CPSIE		I
			BX			LR
			
			
;Title: PutChar Subroutine
;Functionality: outputs a character to the terminal
;Input: Character ASCII value in R0
;Output: No outputs to registers, but output to terminal
;Register Modification List: R0, stores value into UART data register
PutChar     PROC		{R0-R14}
			PUSH		{R1-R3,LR}
			LDR			R1,=TxRecord

LoopPutC	CPSID		I
			BL			Enqueue
			CPSIE		I
			BCS			LoopPutC
			
			MOVS		R1,#UART0_C2_TI_RI
			LDR			R0,=UART0_BASE
			STRB		R1,[R0,#UART0_C2_OFFSET]
			POP			{R1-R3,PC}
			BX			LR
			ENDP


;Title: Get Character Subroutine
;Functionality: Attempts to Dequeue a character from the Rx Queue until it can
;Input: Input is taken from terminal Rx Queue
;Output: ASCII value of input character
;Register Modification List: R0 -> ASCII value
GetChar		PROC		{R1-R14}
			PUSH		{R1-R3}
			LDR			R1,=RxRecord
	
LoopGetC    CPSID		I
			BL			Dequeue
			CPSIE		I
			BCS			LoopGetC
	
			POP			{R1-R3}
			BX			LR
			ENDP

;DIVU SUBROUTINE
;Function: Divides the contents of Register 1 by Register 0
;Input: R1 - Dividend, R0 - Divisor
;Output: R1 - Remainder, R0 - Quotient
;Modifies: R0 + R1
                

DIVU		PROC	{R2-R14}		;Perserves Registers R2 and R3
			PUSH	{R2-R4}		;Pushes R2 and R3 onto the stack
			MOVS	R2,#0		;Initializes Quotient
			CMP		R0,#0		;Compares The divisor to 0
			BEQ		INVALID		;Branches if the divisor is 0
			CMP 	R1,#0
			BEQ		ZERO
WHILE		CMP		R1,R0		;Top of While loop 
			BLO		ENDWHILE	;Is the divisor greater than the dividend?
			SUBS	R1,R1,R0	;Dividend - Divisor = Dividend or R1-R0 => R1 
			ADDS	R2,#1		;Quotient++ or R3 + 1 => R3
			B		WHILE		;Return to top of loop

ZERO		MOVS	R0,#0
			
ENDWHILE	MRS		R3,APSR		;Resets the C Flag 
			MOVS	R4,#0x20	;"""
			LSLS	R4,R4,#24	;"""
			BICS	R3,R3,R4	;"""
			MSR		APSR,R3		;"""
INVALID    
			MOVS	R0,R2		;Moves the quotient to the correct register it should be returned in
V			POP		{R2-R4}		;Returns registers R2-R3 to their original state
			BX		LR			;Ends the Subroutine
            ENDP
            
;Get String Subroutine
;Function: Reads a string from the terminal keyboard to memory starting at the address in R0 ;and adds null termination 
;Input: R0 -> address to start the string
;R1 -> buffer capacity
;Output: String to address in R0
;Uses: GetChar, PutChar

GetString	PROC        {R1-R14}
            PUSH		{R2-R7, LR}
            
            MOVS		R3,#0			;Counter = 0
            MOVS        R6,#0x1F        ;For checking if the char is a control char
            MOVS		R2,R0			;Moves pointer to R2, since R0 is changed
            MOVS        R7,#0x7F
LoopGS		BL		    GetChar		    ;Gets first character
            CMP		    R0,R6   		;If (Character == control char):
            BLT		    CheckChar		;	branches to carriage check
            CMP         R0,R7
            BEQ         LoopGS
            BL          PutChar 
            STRB		R0,[R2,#0]		;Stores the first char
            ADDS		R2,#1			;StrPointer++
            ADDS		R3,#1			;Counter++
            CMP		    R3,R1			;If (Counter = Buffer capacity):
            BEQ		    Buffer  		;	End Loop
            B		    LoopGS	

CheckChar   CMP         R0,#0x0D        ;If a character is in the control character range of 0x00-0x1F it branches here
            BEQ         EndString       ;If that control character is a carriage return it ends if it isn't the main loop continues
            B           LoopGS

Buffer      BL          GetChar         ;If the buffer capacity is reached the program waits in this loop until the carraige return is entered
            CMP         R0,#0x0D
            BEQ         EndString
            B           Buffer

EndString	
            MOVS		R5,#0x00			    ;StrPointer = 0
            STRB		R5,[R2,#0]              ;Store null termination char
            POP		    {R2-R7,PC}
            BX		    LR
            ENDP

;Put String Subroutine
;Displays a null-terminated string from memory,
;starting at the address where R0 points, to the
;terminal screen.
;Parameters
;Input:R0 :  Pointer to source string
;Modify: APSR
;Uses:
;  PutChar

PutString	PROC        {R1-R13,LR}
            PUSH	 	{R2,LR}		    ;Preserve Registers R2 and the LR

            MOVS		R2,R0			;Load the pointer to R2
LoopPS		LDRB		R0,[R2,#0]			;Load the first character to R0
            CMP		    R0,#0			;Check if character == 0:
            BEQ		    EndIf			;	Then end
            BL		    PutChar		    ;put the current char
            ADDS		R2,#1			; Advance the pointer
            B		    LoopPS			;Branch to top of loop

EndIf       POP		    {R2,PC}
            BX		    LR
            ENDP

;PutNumU subroutine
;function: print to the terminal screen the text decimal representation of the unsigned word value ;in R0
;Input: R0 -> word value
;Output: text representation to terminal screen
;Uses: DIVU

PutNumU		PROC	    {R1-R14}
			PUSH		{R1-R3,LR}

			MOVS		R1,R0		    ;DIVU is R1 / R0 so num has to be in R1
            LDR         R2,=Temp        ;Variable that stores the digits backwards
            LDR         R3,=Temp        ;Used to hold the pointer to the start of the num for later use
            SUBS        R3,#1           ;Sets R3 = to the begginning of Temp -1
            
LoopPN		MOVS		R0,#0xA	        ;R0 is divisor 
            BL		    DIVU            ;Branches to DIVU
            ADDS	   	R1,#0x30        ;convert remainder to ascii equivalent
            STRB        R1,[R2,#0]      ;Store MSB digit
            CMP         R0,#0           ;if (quotient == 0)
            BEQ         SecondLoop      ;   End this part
            MOVS        R1,R0           ;Move quotient to divisor register
            ADDS        R2,#1           ;Advance the temp pointer
            B		    LoopPN

SecondLoop  CMP         R2,R3           ;Check if the current pointer in R2 has returned to the start of the variable
            BEQ         EndSub          ;If it does it ends the subroutine
            MOVS        R0,R2           ;Moves the pointer from R2 -> R0
            LDRB        R0,[R2,#0]      ;Loads the current pointed to value to R2
            BL          PutChar         ;Prints the character
            SUBS        R2,#1           ;Pointer--
            B           SecondLoop

EndSub      POP		    {R1-R3,PC}
            BX		    LR
            ENDP

;InitQueue
;Function: initializes the queue record structure at the address in R1for the empty
;queue buffer at the address in R0 of size, (i.e., character capacity), given in
;R2
;Input: R0: Address of queue buffer, R1: Address of Queue record structure, R2: Size
;Output: Initialized Queue and record structure at address in R1 and R2
;Registers Modified: NONE

InitQueue		PROC			{R0-R14}
				PUSH			{R0-R4}

				;Initiliazes the Queue Record Structure
				STR			R0,[R1,#IN_PTR]			;Initializes all of the pointers to the
				STR			R0,[R1,#OUT_PTR]		;beginning of the queue
				STR			R0,[R1,#BUF_STRT]	
				ADDS		R4,R2,R0				;Calculates address of Beginning of Queue + Size
				STR			R4,[R1,#BUF_PAST]		;Puts that value in to Buffer Past
				STRB		R2,[R1,#BUF_SIZE]		;Stores max buffer size to the correct addres
				MOVS		R3,#0					
				STRB		R3,[R1,#NUM_ENQD]		;Initiliazes the Number Enqueued to 0 for an empty Queue
				
				POP			{R0-R4}
				BX			LR
				ENDP
				
				
;Enqueue (EnQ)
;********************************************
;If the queue (whose queue record structure’s address is in R1 is not full, 
;enqueues the character from R0 to the queue and reports success by 
;returning with the C flag ;cleared, (i.e., 0) 
;otherwise only reports failure ;by returning with the C flag set, (i.e., 1).
; Input:  R0:  Character to enqueue
;         R1:  Address of queue record structure
; Output:  PSR C flag:  Success(0) or Failure (1)
; Modify:  APSR
; All other registers remain unchanged on return

Enqueue		PROC		{R0-R14}
			PUSH		{R0-R7,LR}
			
			LDRB		R2,[R1,#NUM_ENQD]	;R2 <- # enqueued
			LDRB		R3,[R1,#BUF_SIZE]	;R3 <- Queue Size
			CMP			R2,R3				;If R2 > R3:
			BGE			FailureE				; 	Queue is full and branch to set C flag
			LDR			R4,[R1,#IN_PTR]		;R4 <- In Pointer
			STRB 		R0,[R4,#0]			;Character -> In Pointer
			ADDS		R2,R2,#1			;Num Enqueued ++
			STRB 		R2,[R1,#NUM_ENQD]	;Store the incrememnted value back
			ADDS		R4,R4,#1			;In pointer ++
			LDR			R5,[R1,#BUF_PAST]	;R5 <- Buffer past address
			CMP			R4,R5				;If (In pointer => Buffer Past)
			BGE			WrapE				;		Wrap the in pointer
			STR			R4,[R1,#IN_PTR]		;Else: Incremented Pointer -> R1 + inpointer
			B			SucEnq


WrapE		LDR			R6,[R1,#BUF_STRT]	;R6 <- Start of Queue Address
			STR			R6,[R1,#IN_PTR]		;In Pointer = Start of Queue
			B			SucEnq

			;Set the C Flag
FailureE	MRS   		R2,APSR
			MOVS  		R3,#0x20
			LSLS  		R3,R3,#24
			ORRS  		R2,R2,R3
			MSR   		APSR,R2
			B 	  		EndEnq

			;Clear the C Flag
SucEnq		MRS   		R0,APSR
			MOVS  		R1,#0x20
			LSLS  		R1,R1,#24
			BICS  		R0,R0,R1
			MSR   		APSR,R0
			
EndEnq		POP			{R0-R7,PC}
			BX			LR
			ENDP
			

;Dequeue (DeQ) 
;**********************************************************
;If the queue (whose queue record structure’s address is in R1) is not empty, dequeues  
;a character from the queue to R0 and reports success by returning with the C flag 
;cleared, (i.e., 0)
; otherwise only reports failure by returning with the C flag set, (i.e., 1). 
; Input:  R1:  Address of queue record structure 
; Output:  R0:  Character dequeued 
;           PSR C flag:  Success(0) or Failure (1)
; Modify:  R0; APSR 
; All other registers remain unchanged on return 

Dequeue		PROC		{R0-R14}
			PUSH		{R1-R7,LR}
			
			LDRB		R2,[R1,#NUM_ENQD]		;R2 <- Num in Queue
			CMP			R2,#0				
			BEQ			FailureD				;If (R2 == 0) -> Queue is empty
			LDR			R3,[R1,#OUT_PTR]		;R3 <- Out Pointer
			LDRB		R0,[R3,#0]				;R0 <- Element at Out Pointer
			SUBS		R2,R2,#1				;Num Enqueued --
			STRB		R2,[R1,#NUM_ENQD]		;Store back new Number Enqueued
			ADDS		R3,R3,#1				;Out Pointer ++
			LDR			R4,[R1,#BUF_PAST]		;R4 <- Buffer Past Address
			CMP			R3,R4
			BGE			WrapD					;If (R3 => R4) -> Out Pointer is Past the end of the Queue
			STR			R3,[R1,#OUT_PTR]		;Incremeneted Out Pointer -> Queue Record Out Pointer
			B			SucDeq
			
			;Wraps the Out pointer to the beginning of the queue
WrapD		LDR			R5,[R1,#BUF_STRT]
			STR			R5,[R1,#OUT_PTR]
			B			SucDeq
			
			;Set the C Flag
FailureD	MRS   		R2,APSR
			MOVS  		R3,#0x20
			LSLS  		R3,R3,#24
			ORRS  		R2,R2,R3
			MSR   		APSR,R2
			B 	  		EndDec

			;Clear the C Flag
SucDeq		MRS   		R2,APSR
			MOVS  		R3,#0x20
			LSLS  		R3,R3,#24
			BICS  		R2,R2,R3
			MSR   		APSR,R2
			
EndDec		POP			{R1-R7,PC}
			BX			LR
			ENDP


;PutNumHex
;Function: subroutine prints to the terminal screen the text 
;hexadecimal representation of the unsigned word value in R0. 
;Input: R0: Unsigned Word Value to be printed
;Output: Word Value to Terminal Screen
;Registers Modified: PSR and nothing else

PutNumHex	PROC		{R0-R14}
			PUSH		{R0-R5,LR}
			
            MOVS        R5,#0x1C
			MOVS		R4,#0x7		;Initialize a counter for the loop
			MOVS		R3,R0		;Preserve the number in R3
			LDR			R1,=0xF0000000	;R1 <- Mask
HexLoop		CMP			R4,#0		;8 is the max amount of iterations
			BLT			EndHex		
            MOVS        R0,R3
			ANDS		R0,R0,R1	;AND the number and the mask
			LSRS		R0,R0,R5	;Shift the result from the AND To the LSB position	
            CMP         R0,#0xA
            BLT         Number
            ADDS        R0,R0,#0x37 ;Convert to ASCII Equivalent
NumberSkip  BL          PutChar
			LSLS		R3,R3,#4	;Shift the word value to put the next digit to be printed in the MSB spot
			SUBS		R4,R4,#1	;decrement the counter
			B			HexLoop

Number      ADDS        R0,R0,#0x30
            B           NumberSkip
            
EndHex		POP			{R0-R5,PC}
			BX			LR
			ENDP
			


;PutNumUB
;Function: Prints to the terminal screen the text decimal 
;representation of the unsigned byte value in R0.
;Input: R0: unsigned byte value
;Output: Prints the byte value to the terminal screen
;Registers Modified: PSR and nothing else

PutNumUB	PROC		{R0-R14}
			PUSH		{R0-R1,LR}
			
			MOVS		R1,#0x000000FF  ;R1 <- Mask
			ANDS		R0,R0,R1        ;R0 <- 0x000000xx
			BL			PutNumU         ;Prints xx
			
			POP			{R0-R1,PC}
			BX			LR
			ENDP
 
;NewLine
;Function: Outputs a new line without altering any registers
;Input: None
;Output: None
;Registers modified: PC

NewLine     PROC       {R0-R7}
            PUSH        {R0,LR}
            
            MOVS        R0,#0x0D    ;R0 <- Carriage Return
            BL          PutChar
            MOVS        R0,#0x0A    ;R0 <- Newline
            BL          PutChar
            
            POP         {R0,PC}
            BX          LR
            ENDP


;PrintQueue
;Function: Prints the Queue in order of Out-Pointer -> In-Pointer Without Changing any registers
;Input: NONE
;Output: Queue Contents to Terminal
;Registers Modified: PSR
PrintQueue  PROC        {R0-R14}
            PUSH        {R0-R5,LR}

            MOVS        R0,#0x3E                
            BL          PutChar                 ;First Print a '>' character
            LDR         R1,=QRecord
            LDRB	    R2,[R1,#NUM_ENQD]		;R2 <- Num in Queue
			CMP			R2,#0				
			BEQ			EndPLoop			    ;If (R2 == 0) -> Queue is empty
		    LDR			R3,[R1,#OUT_PTR]		;R3 <- Out Pointer
            LDR         R4,[R1,#IN_PTR]         ;R4 <- In Pointer
            LDR			R5,[R1,#BUF_PAST]		;R5 <- Buffer Past Address

PrintLoop   CMP         R2,#0                   ;If no elements are left end the loop
            BEQ         EndPLoop
            LDRB		R0,[R3,#0]				;R0 <- Element at Out Pointer
            BL          PutChar
			ADDS		R3,R3,#1				;Out Pointer ++
			SUBS        R2,R2,#1                ;Counter--   
            CMP			R3,R5               
			BGE			Wrap					;If (R3 => R5) -> Out Pointer is Past the end of the Queue
			B			PrintLoop
            
Wrap		LDR			R5,[R1,#BUF_STRT]
			MOVS        R3,R5
			B			PrintLoop

EndPLoop    MOVS        R0,#0x3C
            BL          PutChar
            POP         {R0-R5,PC}
            BX          LR
            ENDP
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
		


	
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
;Management record structure field displacements 
IN_PTR    EQU    0 
OUT_PTR   EQU    4 
BUF_STRT  EQU    8 
BUF_PAST  EQU    12 
BUF_SIZE  EQU    16 
NUM_ENQD  EQU    17 
; Queue structure sizes 
Q_BUF_SZ  EQU    80        ;Room for 4 characters 
Q_REC_SZ  EQU    18        ;Management record size 

            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
Prompt          DCB     "Type a Queue Command (D,E,H,P,S):",0
SuccessPrompt   DCB     "Success!",0
FailurePrompt   DCB     "Failure!",0
EnQ             DCB     "Character to be Enqueued:",0
StatusPrompt    DCB     " Status:",0
In              DCB     "In=0x",0
Out             DCB     "   Out=0x",0
Num             DCB     "   Num=",0
HelpPrompt      DCB     "d (dequeue), e (enqueue), h (help), p (print), s (status)",0
InvalidCommand  DCB     "Invalid Command",0
Test			DCB		"HI",0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;Queue structures 	
TxBuffer   SPACE  Q_BUF_SZ  ;Queue contents 
			ALIGN
TxRecord   SPACE  Q_REC_SZ  ;Queue management record	
			ALIGN
RxBuffer   SPACE  Q_BUF_SZ	
			ALIGN
RxRecord   SPACE  Q_REC_SZ	
			ALIGN
QBuffer	   SPACE  4	
			ALIGN
QRecord	   SPACE  18		
			ALIGN
Temp       SPACE  2
			ALIGN
;>>>>>   end variables here <<<<<
            ALIGN
            END