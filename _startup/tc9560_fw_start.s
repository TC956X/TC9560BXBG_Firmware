;============================================================================== 
; PROJECT: TC9560
; Copyright (C) 2018  Toshiba Electronic Devices & Storage Corporation
;
; This program is free software; you can redistribute it and/or
; modify it under the terms of the GNU General Public License
; as published by the Free Software Foundation; either version 2
; of the License, or (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
;============================================================================== 

; History:   
;		18 July 2016: Revision: 1.0
;

;
; IROM1: Base: 0x10000000, Max: 0x00012000,	72K
; IRAM1: Base: 0x10012000, Max: 0x00004000, 16K
; HEAP:  Base: 0x10016000, Max: 0x00001000, 4K
; STACK: Base: 0x10017000, Max: 0x00001000, 4K
;


;<h> STACK CONFIGURATION
;<o> Stack Size (in Bytes) 
;<o> stack size  4KB (0x1000) using SRAM at 0x10017000 
;<o> stack pointer intialized to 0x10017FFF desending stack
;</h>

Stack_Size		EQU		0x00001000

				AREA	STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem		SPACE	Stack_Size
__initial_sp	EQU		0x6FFFC
;__initial_sp	
;EQU		0x00000000
	;0x1002FFFC

;<h> HEAP CONFIGURATION
;<o> Heap Size (in Bytes)
;<0> Heap size  4KB (0x1000) using SRAM at 0x10016000
;</h>

;Heap_Size		EQU		32768
Heap_Size		EQU			0x44000	
;Heap_Size		EQU		0x00000000		

				AREA	HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base		
;EQU 	0x10016000
Heap_Mem		SPACE	Heap_Size
__heap_limit	
;EQU 	0x10016FFC


				PRESERVE8
				THUMB

;/**************************************************************************/
;
; Vector Table residing into ROM at Address 0, 
;
 
;				AREA	ROM_RESET, DATA, READONLY			
;				EXPORT	__ROM_Vectors

;__ROM_Vectors	DCD		__initial_sp			; Top of Stack
;				DCD		Reset_Handler		    ; Reset Handler

;				AREA	ROM_RESET_CODE, CODE, READONLY
;/**************************************************************************/
;
; Vector Table Mapped to Address 0 at Reset

				AREA	RESET, DATA, READONLY
				EXPORT	__Vectors

__Vectors		DCD		__initial_sp			; Top of Stack
				DCD		Reset_Handler			; Reset Handler
				DCD		NMI_Handler				; NMI Handler
				DCD		HardFault_Handler		; Hard Fault Handler
				DCD		MemManage_Handler		; MPU Fault Handler
				DCD		BusFault_Handler		; Bus Fault Handler
				DCD		UsageFault_Handler		; Usage Fault Handler
				DCD		0						; Reserved
				DCD		0						; Reserved
				DCD		0						; Reserved
				DCD		0						; Reserved
				DCD		SVC_Handler				; SVCall Handler
				DCD		DebugMon_Handler		; Debug Monitor Handler
				DCD		0						; Reserved
				DCD		PendSV_Handler			; PendSV Handler
				DCD		SysTick_Handler			; SysTick Handler
			
				; External Interrupts
				
                DCD     INT_INTR0             ;  0, INT0       [  0] Interrupt pin 0                                             
                DCD     INT_INTR1             ;  1, INT1       [  1] Interrupt pin 1                                             
                DCD     INT_INTR2             ;  2, INT2       [  2] Interrupt pin 2                                             
                DCD     INT_INTR3             ;  3, INT3       [  3] Interrupt pin 3                                             
                DCD     INT_INTR4             ;  4, INT4       [  4] Interrupt pin 4                                             
                DCD     INT_INTR5             ;  5, INT5       [  5] Interrupt pin 5                                             
                DCD     INT_INTR6             ;  6, INT6       [  6] Interrupt pin 6                                             
                DCD     INT_INTR7             ;  7, INT7       [  7] Interrupt pin 7  HSIC Interrupt          
                DCD     INT_INTR8             ;  8, INT8       [  7] Interrupt pin 7  
                DCD     INT_INTR9             ;  9, INT9       [  7] Interrupt pin 7            
                DCD     INT_INTR10             ;  10, INT10       [  7] Interrupt pin 7  
                DCD     INT_INTR11             ;  11, INT11       [  7] Interrupt pin 7            
                DCD     INT_INTR12             ;  12, INT12       [  7] Interrupt pin 7  EMAC TXCH0	
                DCD     INT_INTR13             ;  13, INT13       [  7] Interrupt pin 7  EMAC TXCH1          										
                DCD     INT_INTR14             ;  13, INT14       [  7] Interrupt pin 7  EMAC TXCH2          															
                DCD     INT_INTR15             ;  13, INT15       [  7] Interrupt pin 7  EMAC TXCH3          										
                DCD     INT_INTR16             ;  13, INT16       [  7] Interrupt pin 7  EMAC TXCH4          										
                DCD     INT_INTR17             ;  13, INT17       [  7] Interrupt pin 7  EMAC RXCH0          										
                DCD     INT_INTR18             ;  13, INT18       [  7] Interrupt pin 7  EMAC RXCH1          															
                DCD     INT_INTR19             ;  13, INT19       [  7] Interrupt pin 7  EMAC RXCH2          										
                DCD     INT_INTR20             ;  13, INT20       [  7] Interrupt pin 7  EMAC RXCH3          										
                DCD     INT_INTR21             ;  13, INT21       [  7] Interrupt pin 7  EMAC RXCH4          															
                DCD     INT_INTR22             ;  13, INT22       [  7] Interrupt pin 7  EMAC RXCH5          																				



                DCD     OS_CPU_IntHandler             ;  24, INT8       [  8] Interrupt pin 8                                             
                DCD     OS_CPU_IntHandler             ;  25, INT9       [  9] Interrupt pin 9                                             
                DCD     OS_CPU_IntHandler             ;  26, INTA       [ 10] Interrupt pin A                                             
                DCD     OS_CPU_IntHandler             ;  27, INTB       [ 11] Interrupt pin B                                             
                DCD     OS_CPU_IntHandler             ;  28, INTC       [ 12] Interrupt pin C                                             
                DCD     OS_CPU_IntHandler             ;  29, INTD       [ 13] Interrupt pin D                                             
                DCD     OS_CPU_IntHandler             ;  30, INTE       [ 14] Interrupt pin E                                             
                DCD     OS_CPU_IntHandler             ;  31, INTF       [ 15] Interrupt pin F                                             
                DCD     OS_CPU_IntHandler             ;  32, INTRX0     [ 16] Serial reception (channel.0)                                
                DCD     OS_CPU_IntHandler             ;  33, INTTX0     [ 17] Serial transmission (channel.0)                             
                DCD     OS_CPU_IntHandler             ;  34, INTRX1     [ 18] Serial reception (channel.1)                                
                DCD     OS_CPU_IntHandler             ;  35, INTTX1     [ 19] Serial transmission (channel.1)                             
                DCD     OS_CPU_IntHandler             ;  36, INTRX2     [ 20] Serial reception (channel.2)                                
                DCD     OS_CPU_IntHandler             ;  37, INTTX2     [ 21] Serial transmission (channel.2)                             
                DCD     OS_CPU_IntHandler             ;  38, INTRX3     [ 22] Serial reception (channel.3)                                
                DCD     OS_CPU_IntHandler             ;  39, INTTX3     [ 23] Serial transmission (channel.3)                             
                DCD     OS_CPU_IntHandler             ;  40, INTUART4   [ 24] FULL UART(channel.4)                                        
                DCD     OS_CPU_IntHandler             ;  41, INTUART5   [ 25] FULL UART(channel.5)                                        
                DCD     OS_CPU_IntHandler             ;  42, INTSBI0    [ 26] Serial bus interface 0                                      
                DCD     OS_CPU_IntHandler             ;  43, INTSBI1    [ 27] Serial bus interface 1                                      
                DCD     OS_CPU_IntHandler             ;  44, INTSBI2    [ 28] Serial bus interface 2                                      
                DCD     OS_CPU_IntHandler             ;  45, INTSSP0    [ 29] SPI serial interface 0                                      
                DCD     OS_CPU_IntHandler             ;  46, INTSSP1    [ 30] SPI serial interface 1                                      
                DCD     OS_CPU_IntHandler             ;  47, INTSSP2    [ 31] SPI serial interface 2                                      
                DCD     OS_CPU_IntHandler             ;  48, INTUSBH    [ 32] USB Host Interrupt                                          
                DCD     OS_CPU_IntHandler             ;  49, INTUSBD    [ 33] USB Device Interrupt                                        
                DCD     OS_CPU_IntHandler             ;  50, INTUSBWKUP [ 34] USB WakeUp                                                  
                DCD     OS_CPU_IntHandler             ;  51, INTCANRX   [ 35] CAN RX                                                      
                DCD     OS_CPU_IntHandler             ;  52, INTCANTX   [ 36] CAN TX                                                      
                DCD     OS_CPU_IntHandler             ;  53, INTCANGB   [ 37] CAN STAUTS                                                  
                DCD     OS_CPU_IntHandler             ;  54, INTETH     [ 38] EtherNET Interrupt                                          
                DCD     OS_CPU_IntHandler             ;  55, INTETHWK   [ 39] EtherNET(magic packet detection) interrupt                  
                DCD     OS_CPU_IntHandler             ;  56, INTADAHP   [ 40] Highest priority AD conversion complete interrupt (channel.A)
                DCD     OS_CPU_IntHandler             ;  57, INTADAM0   [ 41] AD conversion monitoring function interrupt 0(channel.A)    
                DCD     OS_CPU_IntHandler             ;  58, INTADAM1   [ 42] AD conversion monitoring function interrupt 1(channel.A)    
                DCD     OS_CPU_IntHandler             ;  59, INTADA     [ 43] AD conversion interrupt(channel.A)                          
                DCD     OS_CPU_IntHandler             ;  60, INTADBHP   [ 44] Highest priority AD conversion complete interrupt (channel.B)
                DCD     OS_CPU_IntHandler             ;  61, INTADBM0   [ 45] AD conversion monitoring function interrupt 0(channel.B)    
                DCD     OS_CPU_IntHandler             ;  62, INTADBM1   [ 46] AD conversion monitoring function interrupt 1(channel.B)    
                DCD     OS_CPU_IntHandler             ;  63, INTADB     [ 47] AD conversion interrupt(channel.B)                          
                DCD     OS_CPU_IntHandler             ;  64, INTEMG0    [ 48] PMD0 EMG interrupt (MPT0)                                   
                DCD     OS_CPU_IntHandler             ;  65, INTPMD0    [ 49] PMD0 PWM interrupt (MPT0)                                   
                DCD     OS_CPU_IntHandler             ;  66, INTENC0    [ 50] PMD0 Encoder input interrupt (MPT0)                         
                DCD     OS_CPU_IntHandler             ;  67, INTEMG1    [ 51] PMD1 EMG interrupt (MPT1)                                   
                DCD     OS_CPU_IntHandler             ;  68, INTPMD1    [ 52] PMD1 PWM interrupt (MPT1)                                   
                DCD     OS_CPU_IntHandler             ;  69, INTENC1    [ 53] PMD1 Encoder input interrupt (MPT1)                         
                DCD     OS_CPU_IntHandler             ;  70, INTMTEMG0  [ 54] 16-bit MPT0 IGBT EMG interrupt                              
                DCD     OS_CPU_IntHandler             ;  71, INTMTPTB00 [ 55] 16-bit MPT0 IGBT period/ TMRB compare match detection 0     
                DCD     OS_CPU_IntHandler             ;  72, INTMTTTB01 [ 56] 16-bit MPT0 IGBT trigger/ TMRB compare match detection 1    
                DCD     OS_CPU_IntHandler             ;  73, INTMTCAP00 [ 57] 16-bit MPT0 input capture 0                                 
                DCD     OS_CPU_IntHandler             ;  74, INTMTCAP01 [ 58] 16-bit MPT0 input capture 1                                 
                DCD     OS_CPU_IntHandler             ;  75, INTMTEMG1  [ 59] 16-bit MPT1 IGBT EMG interrupt                              
                DCD     OS_CPU_IntHandler             ;  76, INTMTPTB10 [ 60] 16-bit MPT1 IGBT period/ TMRB compare match detection 0     
                DCD     OS_CPU_IntHandler             ;  77, INTMTTTB11 [ 61] 16-bit MPT1 IGBT trigger/ TMRB compare match detection 1    
                DCD     OS_CPU_IntHandler             ;  78, INTMTCAP10 [ 62] 16-bit MPT1 input capture 0                                 
                DCD     OS_CPU_IntHandler             ;  79, INTMTCAP11 [ 63] 16-bit MPT1 input capture 1                                 
                DCD     OS_CPU_IntHandler             ;  80, INTMTEMG2  [ 64] 16-bit MPT2 IGBT EMG interrupt     
								DCD     OS_CPU_IntHandler             ;  81									
 								DCD     OS_CPU_IntHandler             ;  82									
 								DCD     OS_CPU_IntHandler             ;  83									
 								DCD     OS_CPU_IntHandler             ;  84									
 								DCD     OS_CPU_IntHandler             ;  85									
 								DCD     OS_CPU_IntHandler             ;  86									
 								DCD     OS_CPU_IntHandler             ;  87									
 								DCD     OS_CPU_IntHandler             ;  88									
 								DCD     OS_CPU_IntHandler             ;  89
								DCD     OS_CPU_IntHandler             ;  90								
								DCD     OS_CPU_IntHandler             ;  91									
					
;				DCD		INTGPIO09_IRQHandler        ; 0:  External interrupt input (GPIO09)
;				DCD		INTGPIO10_IRQHandler		; 1:  External interrupt input (GPIO10)
			
				
				AREA	|.text|, CODE, READONLY


; Reset Handler

Reset_Handler	PROC
				EXPORT	Reset_Handler				[WEAK]
				IMPORT	__main
				LDR		R13, =__initial_sp
				LDR		R0, =__main
				BX		R0
				ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler		PROC
				EXPORT	NMI_Handler			[WEAK]
				IMPORT	NMI_isr
				LDR		R0, =NMI_isr
				BX		R0
				ENDP
HardFault_Handler\
				PROC
				EXPORT	HardFault_Handler		[WEAK]
				IMPORT	HardFault_isr
				LDR		R0, =HardFault_isr
				BX		R0
				ENDP
MemManage_Handler\
				PROC
				EXPORT	MemManage_Handler		[WEAK]
				IMPORT	MemManage_isr
				LDR		R0, =MemManage_isr
				BX		R0
				ENDP 	
BusFault_Handler\
				PROC
				EXPORT	BusFault_Handler		[WEAK]
				IMPORT	BusFault_isr
				LDR		R0, =BusFault_isr
				BX		R0
				ENDP
UsageFault_Handler\
				PROC
				EXPORT	UsageFault_Handler		[WEAK]
				IMPORT	UsageFault_isr
				LDR		R0, =UsageFault_isr
				BX		R0
				ENDP
SVC_Handler     PROC
				EXPORT	SVC_Handler			[WEAK]
				IMPORT	SVC_isr
				LDR		R0, =SVC_isr
				BX		R0
				ENDP
DebugMon_Handler\
				PROC
				EXPORT	DebugMon_Handler		[WEAK]
				IMPORT	DebugMon_isr
				LDR		R0, =DebugMon_isr
				BX		R0
				ENDP
PendSV_Handler  PROC
				EXPORT	PendSV_Handler			[WEAK]
				IMPORT	PendSV_isr
				LDR		R0, =PendSV_isr
				BX		R0
				ENDP
SysTick_Handler PROC
				EXPORT	SysTick_Handler			[WEAK]
				IMPORT	SysTick_isr
				LDR		R0, =SysTick_isr
				BX		R0
				ENDP

; One Default_Handler for all external interrupts (infinite loops which can be modified)
Default_Handler PROC

				EXPORT     INT_INTR0             [WEAK];  16, INT0       [  0] Interrupt pin 0                                             
                EXPORT     INT_INTR1             [WEAK];  17, INT1       [  1] Interrupt pin 1                                             
                EXPORT     INT_INTR2             [WEAK];  18, INT2       [  2] Interrupt pin 2                                             
                EXPORT     INT_INTR3             [WEAK];  19, INT3       [  3] Interrupt pin 3                                             
                EXPORT     INT_INTR4             [WEAK];  20, INT4       [  4] Interrupt pin 4                                             
                EXPORT     INT_INTR5             [WEAK];  21, INT5       [  5] Interrupt pin 5                                             
                EXPORT     INT_INTR6             [WEAK];  22, INT6       [  6] Interrupt pin 6                                             
                EXPORT     INT_INTR7             [WEAK];  23, INT7       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR8             [WEAK];  23, INT8       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR9             [WEAK];  23, INT9       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR10             [WEAK];  23, INT10       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR11             [WEAK];  23, INT11       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR12             [WEAK];  23, INT12       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR13             [WEAK];  23, INT13       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR14             [WEAK];  23, INT14       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR15             [WEAK];  23, INT15       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR16             [WEAK];  23, INT16       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR17             [WEAK];  23, INT17       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR18             [WEAK];  23, INT18       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR19             [WEAK];  23, INT19       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR20             [WEAK];  23, INT20       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR21             [WEAK];  23, INT21       [  7] Interrupt pin 7  HSIC Interrupt             				
                EXPORT     INT_INTR22             [WEAK];  23, INT22       [  7] Interrupt pin 7  HSIC Interrupt             				
			
				
INT_INTR0
				LDR		R1, =(global_isr_table + 0)
				MOV		R0, #0
				B       execute_isr
				
INT_INTR1
				LDR		R1, =(global_isr_table + 4)
				MOV		R0, #1
				B       execute_isr				
				
INT_INTR2
				LDR		R1, =(global_isr_table + 8)
				MOV		R0, #2
				B       execute_isr				
				
INT_INTR3
				LDR		R1, =(global_isr_table + 12)
				MOV		R0, #3
				B       execute_isr				

INT_INTR4
				LDR		R1, =(global_isr_table + 16)
				MOV		R0, #4
				B       execute_isr				

INT_INTR5
				LDR		R1, =(global_isr_table + 20)
				MOV		R0, #5
				B       execute_isr				

INT_INTR6
				LDR		R1, =(global_isr_table + 24)
				MOV		R0, #6
				B       execute_isr				

INT_INTR7
				LDR		R1, =(global_isr_table + 28)
				MOV		R0, #7
				B       execute_isr				

INT_INTR8
				LDR		R1, =(global_isr_table + 32)
				MOV		R0, #8
				B       execute_isr				

INT_INTR9
				LDR		R1, =(global_isr_table + 36)
				MOV		R0, #9
				B       execute_isr				

INT_INTR10
				LDR		R1, =(global_isr_table + 40)
				MOV		R0, #10
				B       execute_isr				

INT_INTR11
				LDR		R1, =(global_isr_table + 44)
				MOV		R0, #11
				B       execute_isr				

INT_INTR12
				LDR		R1, =(global_isr_table + 48)
				MOV		R0, #12
				B       execute_isr				

INT_INTR13
				LDR		R1, =(global_isr_table + 52)
				MOV		R0, #13
				B       execute_isr				

INT_INTR14
				LDR		R1, =(global_isr_table + 56)
				MOV		R0, #14
				B       execute_isr				

INT_INTR15
				LDR		R1, =(global_isr_table + 60)
				MOV		R0, #15
				B       execute_isr				

INT_INTR16
				LDR		R1, =(global_isr_table + 64)
				MOV		R0, #16
				B       execute_isr				

INT_INTR17
				LDR		R1, =(global_isr_table + 68)
				MOV		R0, #17
				B       execute_isr				

INT_INTR18
				LDR		R1, =(global_isr_table + 72)
				MOV		R0, #18
				B       execute_isr				

INT_INTR19
				LDR		R1, =(global_isr_table + 76)
				MOV		R0, #19
				B       execute_isr				

INT_INTR20
				LDR		R1, =(global_isr_table + 80)
				MOV		R0, #20
				B       execute_isr				

INT_INTR21
				LDR		R1, =(global_isr_table + 84)
				MOV		R0, #21
				B       execute_isr				

INT_INTR22
				LDR		R1, =(global_isr_table + 88)
				MOV		R0, #22
				B       execute_isr				

OS_CPU_IntHandler
				LDR		R1, =(global_isr_table + 92)
				MOV		R0, #23
				B       execute_isr				



execute_isr 
 				IMPORT	global_isr_table
				IMPORT isr_no
				LDR 	R2, =isr_no
				STR		R0, [R2]
				LDR     R0, [R1]
				BX	R0

				ENDP
				   
				ALIGN


; User Initial Stack & Heap

				IF	:DEF:__MICROLIB
				
				EXPORT	__initial_sp
;				EXPORT	__Stack_Size
                EXPORT  __heap_base
                EXPORT  __heap_limit					
					
				ELSE
					
;                IMPORT  __use_two_region_memory					
				EXPORT	__user_initial_stackheap
__user_initial_stackheap
					 
;                LDR     R0, =  Heap_Mem
;                LDR     R1, =(Stack_Mem + Stack_Size)
;                LDR     R2, = (Heap_Mem +  Heap_Size)
;                LDR     R3, = Stack_Mem
;                BX      LR


				LDR		R3, = Stack_Mem
				BX		LR
					
				ALIGN
				
				ENDIF
					
					
				END

