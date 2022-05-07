/*
 * LCD.h
 *
 *  Created on: Oct 9, 2021
 *      Author: jogde
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#define RS 0x100
#define RW 0x200
#define E  0x400

void LCD_command(unsigned char command);
void LCD_write_char(unsigned char letter);
void LCD_clear(void);
void LCD_newline(void);
void LCD_init(void);

#endif /* SRC_LCD_H_ */
