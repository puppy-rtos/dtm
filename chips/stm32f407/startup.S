.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.section .vectors
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    /* 省略其他中断向量... */

.text
.global Reset_Handler
Reset_Handler:
    /* 初始化堆栈指针 */
    ldr r0, =_estack
    mov sp, r0
    
    /* 调用系统初始化 */
    bl SystemInit
    
    /* 复制.data段 */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    bl memory_copy
    
    /* 清零.bss段 */
    ldr r0, =_sbss
    ldr r1, =_ebss
    bl memory_zero
    
    /* 跳转到main */
    bl main

/* 内存操作函数 */
memory_copy:
    cmp r0, r1
    beq 2f
1:  ldr r3, [r2], #4
    str r3, [r0], #4
    cmp r0, r1
    blt 1b
2:  bx lr

memory_zero:
    cmp r0, r1
    beq 2f
    mov r2, #0
1:  str r2, [r0], #4
    cmp r0, r1
    blt 1b
2:  bx lr

/* 弱定义默认中断处理 */
.weak NMI_Handler
.thumb_set NMI_Handler,Default_Handler

.weak HardFault_Handler
.thumb_set HardFault_Handler,Default_Handler

Default_Handler:
    b .