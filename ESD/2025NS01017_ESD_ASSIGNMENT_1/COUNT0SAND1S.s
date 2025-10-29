	AREA RESET, CODE, READONLY
    ENTRY
START
    LDR R2, =1017      ; Load your number (last 5 digits of ID)
    MOV R0, #0         ; R0 will store the count of 0s
    MOV R1, #0         ; R1 will store the count of 1s

    ; Check if the number is zero to begin with. If so, skip to end.
    CMP R2, #0
    BEQ STOP

LOOP
    ; Check the last bit and update the Z flag
    ANDS R4, R2, #1

    ; If the result was 0 (EQ), increment the zero counter.
    ADDEQ R0, R0, #1
    ; If the result was not 0 (NE), increment the one counter.
    ADDNE R1, R1, #1

    ; Shift the number right by 1 and UPDATE THE FLAGS (critical!)
    MOVS R2, R2, LSR #1

    ; Branch back to LOOP if R2 is Not Equal to zero.
    BNE LOOP

STOP
    B STOP             ; End of program

    END