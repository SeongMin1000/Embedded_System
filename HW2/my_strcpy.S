    AREA asm_func, CODE, READONLY
    EXPORT my_strcpy
my_strcpy
loop                        ; Start of string copy loop
    LDRB    R2, [R0], #1    ; Load 1 byte (character) from src(R0) to R2 and increment pointer (read one character and point to the next)
    STRB    R2, [R1], #1    ; Store the loaded character to dst(R1) and increment pointer (store one character and point to the next location)
    CMP     R2, #0          ; Compare the loaded character with NULL(0) (check if it's the end of string)
    BNE     loop            ; If not NULL, repeat the loop
    BX      LR              ; Return to caller
    END                     ; End of assembly code
