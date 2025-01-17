        .section .data
input_string:   .asciz "Hello World!"

        # tohost/fromhost are data locations used by Spike
        .align 8
tohost: .byte 0
        .align 8
fromhost: .byte 0

        
        .section .text
        .globl _start

_start:
        # Load the address of the input string into a0
        la a0, input_string
        # Your code here!
        # Initiate length of the string
        # andi a0, a0, 0     # a0 = length of the string; initialize to 0
        # li t1, 0              # t1 = index i (initialize to 0)
        li t2, 0              # stores the temporary char
        # Check if the character is a lowercase letter (97 <= string[i] <= 122)
        li t4, 97             # t4 = 'a'
        li t5, 123            # t5 = 'z'

loop_start:
        # Load a byte from the string
        lbu t2, 0(a0)         # t2 = *t0 (load unsigned byte)

        # Check if the character is null ('\0')
        beqz t2, end_program     # If t2 == 0, exit the loop

        # If not, check if current char is larger than z
        bgeu t2, t5, skip

        # Check if current char is smaller than a
        bltu t2, t4, skip

        # Else, decrement current ASCII char by 32 for upper case
        addi t2, t2, -32

        # Store the decremented (Uppercase) value back to the memory
        sb t2, 0(a0)

        # Move to the next character
        addi a0, a0, 1        # a0 = a0 + 1 (next memory location)

        # Repeat the loop
        j loop_start

skip:
        # Move to the next character
        addi a0, a0, 1
        j loop_start

end_program:
        # jump here when you're done
        j end_program
