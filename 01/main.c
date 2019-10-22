#include <stdio.h>

int main ()
{
    // ResetBit
    printf("Result 1: %X\n", ResetBit(0xF, 2));
    printf("Result 2: %X\n", ResetBit(0xA, 0));

    // ResetTwoBits
    printf("Result 3: %X\n", ResetTwoBits(0xFF, 3));
    printf("Result 4: %X\n", ResetTwoBits(0xB7, 3));
    
    // SetBit
    printf("Result 5: %X\n", SetBit(0xA, 0));
    printf("Result 6: %X\n", SetBit(0xE, 2));
    
    // SetTwoBitsTo
    printf("Result 7: %X\n", SetTwoBitsTo(0xFF, 3, 1));
    printf("Result 8: %X\n", SetTwoBitsTo(0xAF, 3, 2));  
    
    return 0;
}

int ResetBit(int x, int p)
{
    return x &= ~(1UL << p);
}

int ResetTwoBits(int x, int p)
{
    return x &= ~(3UL << p); 
}

int SetBit(int x, int p)
{
    return x |= (1UL << p);
}

int SetTwoBitsTo(int x, int p, int n)
{
    return x & ~(3 << p)|(n << p);
}
