    AREA asm_func, CODE, READONLY
    EXPORT my_capitalize
my_capitalize
loop                        ; Start of capitalization loop
    LDRB    R1, [R0]        ; Load 1 byte from src(R0) to R1 (read the character pointed by src)
    CMP     R1, #'a'-1      ; Compare the loaded character with 'a'-1 ('a'-1 < lowercase < 'z'+1, ASCII)
    BLS     cap_skip        ; If less than or equal to 'a'-1, skip capitalization (branch to cap_skip label)
    CMP     R1, #'z'        ; Compare the loaded character with 'z'
    BHI     cap_skip        ; If greater than 'z', skip capitalization (branch to cap_skip label)
    SUB     R1, R1, #32     ; Subtract 32 from lowercase to convert to uppercase (ASCII difference)
    STRB    R1, [R0]        ; Store the converted character back to the original position
cap_skip                    ; Skip capitalization label
    ADD     R0, R0, #1      ; Move string pointer to the next position (skip current character to read the next one)
    CMP     R1, #0          ; Compare character with NULL(0) (check if it's the end of string)
    BNE     loop            ; If not NULL, repeat the loop
    BX      LR              ; Return to caller
    END                     ; End of assembly code
