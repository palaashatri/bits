; Vector Table
    AREA RESET, DATA, READONLY
    EXPORT __Vectors
__Vectors
    DCD 0x20002000      ; Main Stack Pointer Top
    DCD Reset_Handler
    DCD 0, 0, 0, 0, 0, 0, 0, 0, 0 ; Reserved Vectors
    DCD SVC_Handler

; Main Application
    AREA MYCODE, CODE, READONLY
    ENTRY
    EXPORT Reset_Handler
Reset_Handler
    ; Initialize Process Stack Pointer (PSP)
    LDR R0, =0x20001000
    MSR PSP, R0

    ; Switch to unprivileged Thread Mode using PSP
    MOV R0, #2
    MSR CONTROL, R0

    ; Load parameters and call SVC
    LDR R0, =1017       ; Parameter 1 from BITS ID
    LDR R1, =1017       ; Parameter 2 from BITS ID
    SVC #17             ; SVC number from BITS ID

STOP
    B STOP

; SVC Exception Handler
SVC_Handler
    ; Get the PSP value
    TST LR, #4
    MRSNE R0, PSP

    ; Extract the SVC immediate number
    LDR R1, [R0, #24]
    LDRB R1, [R1, #-2]

    ; Load parameters and BITS ID for comparison
    LDR R2, =17
    CMP R1, R2
    LDR R2, [R0, #0]
    LDR R3, [R0, #4]

    ; Perform operation based on comparison
    ADDEQ R2, R2, R3
    SUBNE R2, R2, R3

    ; Store result and return
    STR R2, [R0, #0]
    BX LR

    END