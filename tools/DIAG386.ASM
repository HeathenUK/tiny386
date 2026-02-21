; DIAG386.COM - Emulator Bottleneck Diagnostic
; Designed for tiny386 on ESP32-P4 (Tanmatsu)
;
; Tests specific performance hypotheses:
;   T1: Register ALU baseline     (pure dispatch + execute)
;   T2: 0x0F MOVZX                (two-byte opcode overhead)
;   T3: Sequential memory reads   (D-cache sequential)
;   T4: PUSH/POP pairs            (stack memory pattern)
;   T5: CMP+JNE fused             (macro-op fusion)
;   T6: CALL+RET                  (control flow overhead)
;   T7: Tiny loop via JMPs        (I-cache best case)
;   T8: 32KB-spread loop via JMPs (I-cache worst case)
;   T9: REP MOVSD                 (string fast path)
;
; Key diagnostic ratios:
;   T8/T7 (normalized) = I-cache miss overhead
;   T2/T1 = 0x0F dispatch overhead
;   T5 vs T1 = fusion effectiveness
;   T4 vs T1 = stack memory overhead
;
; Output goes to BOTH DOS screen AND port 0xE9 (host serial debug)
;
; Assemble: nasm -f bin -o DIAG386.COM DIAG386.ASM
; Run in DOS on tiny386

[bits 16]
[org 0x100]
[cpu 486]

%define ITERS      1000000   ; T1-T6 iterations
%define ITERS_IC    500000   ; T7-T8 iterations (T8 slower)
%define ITERS_STR     5000   ; T9: each copies 4KB = 20MB total

; Bochs-style debug console port (emulator writes to host serial)
%define DEBUG_PORT 0xE9

; Buffer addresses within padding region (NOP-filled, before T8 stations)
BUF_SRC equ 0x0E00           ; 4KB source buffer for T3/T9
BUF_DST equ 0x1E00           ; 4KB dest buffer for T9

start:
    ; Ensure ES = DS (for REP MOVS later)
    push ds
    pop es

    mov si, msg_banner
    call print_str

    ;=== T1: Register ALU (ADD EAX,EBX x8) - baseline ===
    mov si, msg_t1
    call print_str
    call start_timer
    mov ecx, ITERS
    xor eax, eax
    mov ebx, 1
.t1:
    add eax, ebx
    add eax, ebx
    add eax, ebx
    add eax, ebx
    add eax, ebx
    add eax, ebx
    add eax, ebx
    add eax, ebx
    dec ecx
    jnz .t1
    call stop_print

    ;=== T2: 0x0F MOVZX (MOVZX EAX,BL x8) - two-byte opcode ===
    mov si, msg_t2
    call print_str
    call start_timer
    mov ecx, ITERS
    xor eax, eax
    mov bl, 42
.t2:
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    movzx eax, bl
    dec ecx
    jnz .t2
    call stop_print

    ;=== T3: Sequential memory reads (MOV EAX,[BX+DI] x4) ===
    mov si, msg_t3
    call print_str
    call start_timer
    mov ecx, ITERS
    mov bx, BUF_SRC
    xor di, di
.t3:
    mov eax, [bx+di]
    mov eax, [bx+di+4]
    mov eax, [bx+di+8]
    mov eax, [bx+di+12]
    add di, 16
    and di, 0FF0h         ; wrap within 4KB, 16-byte aligned
    dec ecx
    jnz .t3
    call stop_print

    ;=== T4: PUSH+POP pairs (x4) ===
    mov si, msg_t4
    call print_str
    call start_timer
    mov ecx, ITERS
    xor eax, eax
.t4:
    push eax
    pop eax
    push eax
    pop eax
    push eax
    pop eax
    push eax
    pop eax
    dec ecx
    jnz .t4
    call stop_print

    ;=== T5: CMP+JNE fused (x4 pairs) ===
    mov si, msg_t5
    call print_str
    call start_timer
    mov ecx, ITERS
    xor eax, eax
    mov ebx, 1            ; EAX != EBX, so JNE always taken
.t5:
    cmp eax, ebx
    jne .t5a
.t5a:
    cmp eax, ebx
    jne .t5b
.t5b:
    cmp eax, ebx
    jne .t5c
.t5c:
    cmp eax, ebx
    jne .t5d
.t5d:
    dec ecx
    jnz .t5
    call stop_print

    ;=== T6: CALL+RET near (x4) ===
    mov si, msg_t6
    call print_str
    call start_timer
    mov ecx, ITERS
.t6:
    call t6_target
    call t6_target
    call t6_target
    call t6_target
    dec ecx
    jnz .t6
    call stop_print

    ;=== T7: Tiny loop via JMPs (all compact, ~60 bytes) ===
    ; 8 stations, each: ADD EAX,EBX + JMP next (adjacent)
    ; Same structure as T8 but all stations within 64 bytes
    mov si, msg_t7
    call print_str
    call start_timer
    mov ecx, ITERS_IC
    xor eax, eax
    mov ebx, 1
    jmp t7_s0

t7_s0: add eax, ebx
       jmp t7_s1
t7_s1: add eax, ebx
       jmp t7_s2
t7_s2: add eax, ebx
       jmp t7_s3
t7_s3: add eax, ebx
       jmp t7_s4
t7_s4: add eax, ebx
       jmp t7_s5
t7_s5: add eax, ebx
       jmp t7_s6
t7_s6: add eax, ebx
       jmp t7_s7
t7_s7: add eax, ebx
       dec ecx
       jnz t7_s0

    call stop_print

    ;=== T8: 32KB-spread loop via JMPs (stations 4KB apart) ===
    ; Same 8 ADDs + JMPs as T7, but spread across 32KB
    ; Difference from T7 = I-cache miss overhead
    mov si, msg_t8
    call print_str
    call start_timer
    mov ecx, ITERS_IC
    xor eax, eax
    mov ebx, 1
    jmp t8_s0              ; jump to first station at 0x3000

t8_done:                   ; stations jump back here when done
    call stop_print

    ;=== T9: REP MOVSD (fast-path string copy) ===
    mov si, msg_t9
    call print_str
    call start_timer
    mov ecx, ITERS_STR
.t9:
    push ecx
    mov si, BUF_SRC
    mov di, BUF_DST
    mov cx, 1024           ; 1024 dwords = 4KB
    cld
    rep movsd              ; NASM generates F3 66 A5
    pop ecx
    dec ecx
    jnz .t9
    call stop_print

    ;=== Summary ===
    mov si, msg_summary
    call print_str

    ; Exit to DOS
    mov ax, 4C00h
    int 21h

;---------------------------------------------------------------
; Utility functions — all output goes to BOTH screen and port 0xE9
;---------------------------------------------------------------

; CALL+RET target for T6
t6_target:
    ret

; Print character in AL to both DOS screen and debug port
emit_char:
    push ax
    push dx
    ; Send to debug port (host serial)
    out DEBUG_PORT, al
    ; Send to DOS screen
    mov dl, al
    mov ah, 2
    int 21h
    pop dx
    pop ax
    ret

; Print null-terminated string at DS:SI to both outputs
print_str:
    push ax
    push si
.next:
    lodsb
    test al, al
    jz .done
    call emit_char
    jmp .next
.done:
    pop si
    pop ax
    ret

; Start timer: save BIOS tick count
start_timer:
    push ax
    push cx
    push dx
    xor ah, ah
    int 1ah                ; CX:DX = tick count
    mov [tick_lo], dx
    mov [tick_hi], cx
    pop dx
    pop cx
    pop ax
    ret

; Stop timer + print elapsed ticks (to both outputs)
stop_print:
    push ax
    push cx
    push dx
    xor ah, ah
    int 1ah                ; CX:DX = current tick count
    sub dx, [tick_lo]
    sbb cx, [tick_hi]
    mov ax, dx             ; low word of elapsed (enough for ~60 min)
    call print_dec
    mov si, msg_ticks
    call print_str
    pop dx
    pop cx
    pop ax
    ret

; Print AX as unsigned decimal to both outputs
print_dec:
    push ax
    push bx
    push cx
    push dx
    xor cx, cx             ; digit count
    mov bx, 10
.div:
    xor dx, dx
    div bx                 ; AX = AX/10, DX = AX%10
    push dx                ; save digit
    inc cx
    test ax, ax
    jnz .div
    ; Right-pad: print leading spaces for alignment (5 chars total)
    mov ax, 5
    sub ax, cx
    jle .nopad
    mov bx, ax
.pad:
    mov al, ' '
    call emit_char
    dec bx
    jnz .pad
.nopad:
    ; Print digits
.prn:
    pop ax                 ; get digit
    add al, '0'
    call emit_char
    loop .prn
    pop dx
    pop cx
    pop bx
    pop ax
    ret

;---------------------------------------------------------------
; Data — null-terminated strings (not $-terminated)
;---------------------------------------------------------------
tick_lo: dw 0
tick_hi: dw 0

msg_banner:
    db 'DIAG386 v1.0 - Emulator Bottleneck Diagnostic', 13, 10
    db '-----------------------------------------------', 13, 10
    db 'T1-T6,T9: 1M iter  T7-T8: 500K iter', 13, 10
    db 'T1,T2,T7,T8: 8 ops/iter  T3-T6: varies', 13, 10, 0
msg_t1: db 'T1 REG-ADD  : ', 0
msg_t2: db 'T2 0F-MOVZX : ', 0
msg_t3: db 'T3 MEM-SEQ  : ', 0
msg_t4: db 'T4 PUSH+POP : ', 0
msg_t5: db 'T5 CMP+JNE  : ', 0
msg_t6: db 'T6 CALL+RET : ', 0
msg_t7: db 'T7 I$-TINY  : ', 0
msg_t8: db 'T8 I$-32K   : ', 0
msg_t9: db 'T9 REP-MOVS : ', 0
msg_ticks: db ' ticks', 13, 10, 0
msg_summary:
    db '-----------------------------------------------', 13, 10
    db '1 tick = 55ms.  Key ratios:', 13, 10
    db '  T8/T7 = I-cache miss cost', 13, 10
    db '  T2/T1 = 0x0F prefix overhead', 13, 10
    db '  T5/T1 = CMP+Jcc fusion gain', 13, 10
    db '  T4/T1 = stack memory overhead', 13, 10
    db '  T3/T1 = memory read overhead', 13, 10, 0

;---------------------------------------------------------------
; Padding to T8 stations
; Also serves as buffer memory for T3 and T9
;---------------------------------------------------------------
times (0x3000 - ($-$$)) db 0x90

;---------------------------------------------------------------
; T8 stations: 8 stations spread 4KB apart = 32KB span
; Each station: ADD EAX,EBX + JMP to next station
; Station 7: ADD + DEC ECX + JNZ station0 + JMP t8_done
;---------------------------------------------------------------

t8_s0:
    add eax, ebx
    jmp t8_s1
times (0x4000 - ($-$$)) db 0x90

t8_s1:
    add eax, ebx
    jmp t8_s2
times (0x5000 - ($-$$)) db 0x90

t8_s2:
    add eax, ebx
    jmp t8_s3
times (0x6000 - ($-$$)) db 0x90

t8_s3:
    add eax, ebx
    jmp t8_s4
times (0x7000 - ($-$$)) db 0x90

t8_s4:
    add eax, ebx
    jmp t8_s5
times (0x8000 - ($-$$)) db 0x90

t8_s5:
    add eax, ebx
    jmp t8_s6
times (0x9000 - ($-$$)) db 0x90

t8_s6:
    add eax, ebx
    jmp t8_s7
times (0xA000 - ($-$$)) db 0x90

t8_s7:
    add eax, ebx
    dec ecx
    jnz t8_s0
    jmp t8_done
