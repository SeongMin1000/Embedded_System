    AREA    asm_func, CODE, READONLY
    EXPORT  factorial

factorial
    PUSH    {R1, LR}        

    CMP     R0, #1          ; if n <= 1
    BLE     base_case       ; return 1

    MOV     R1, R0          ; R1 = n
    SUB     R0, R0, #1      ; n = n - 1
    BL      factorial       ; 재귀 호출

    MUL     R0, R0, R1      ; R0 = R0 * n

    POP     {R1, PC}        

base_case
    MOV     R0, #1          ; n = 1;
    POP     {R1, PC}
    END