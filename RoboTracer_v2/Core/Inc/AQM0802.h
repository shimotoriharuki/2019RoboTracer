#ifndef AQM0802_H
#define AQM0802_H

#include "main.h"
#include <stdarg.h>

void lcd_cmd(uint8_t);
void lcd_data(uint8_t);
void lcd_init(void);
void lcd_clear(void);
void lcd_locate(int,int);
void lcd_print(const char *);
short lcd_printf(const char *, ...);

#endif
