#include <stdio.h>
#include <gpiod.h>

int main(void)
{
    printf("libgpiod version: %s\n", gpiod_version_string());
    return 0;
}
