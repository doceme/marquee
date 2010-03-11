/**
 ******************************************************************************
 *
 * @file       font.h
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Font tables
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef FONT_H
#define FONT_H

// 5x7 Font Table
// ASCII characters 0x20-0x7F (32-127)
const unsigned char font5x7[] = {
	0x3e, 0x17, 0x08, 0x3e, 0x00,// NULL
	0x19, 0x26, 0x0f, 0x02, 0x0f,// SOH
	0x19, 0x26, 0x0d, 0x02, 0x0d,// STX
	0x18, 0x3c, 0x1e, 0x3c, 0x18,// ETX
	0x3e, 0x2a, 0x08, 0x0f, 0x08,// EOT
	0x3e, 0x2a, 0x0f, 0x09, 0x0f,// ENQ
	0x1e, 0x24, 0x1f, 0x02, 0x0d,// ACK
	0x3e, 0x2a, 0x36, 0x0f, 0x00,// BEL
	0x3e, 0x2a, 0x14, 0x0b, 0x00,// BS
	0x3e, 0x2f, 0x0a, 0x05, 0x00,// TAB
	0x3e, 0x02, 0x0f, 0x0a, 0x00,// LF
	0x3c, 0x02, 0x3c, 0x0f, 0x08,// VT
	0x3e, 0x28, 0x00, 0x0f, 0x0a,// FF
	0x1c, 0x22, 0x2f, 0x0a, 0x0d,// CR
	0x19, 0x26, 0x07, 0x08, 0x07,// SO
	0x19, 0x26, 0x08, 0x0f, 0x08,// SI
	0x3f, 0x21, 0x1f, 0x0a, 0x08,// DLE   0x10
	0x3f, 0x21, 0x1e, 0x00, 0x0f,// DC1
	0x3f, 0x21, 0x1e, 0x0b, 0x0c,// DC2
	0x3f, 0x21, 0x1e, 0x0a, 0x0f,// DC3
	0x3f, 0x21, 0x1f, 0x05, 0x09,// DC4
	0x3e, 0x17, 0x0a, 0x3f, 0x04,// NAK
	0x19, 0x26, 0x0c, 0x03, 0x0c,// SYN
	0x3e, 0x2a, 0x0f, 0x0a, 0x05,// ETB
	0x1c, 0x22, 0x27, 0x09, 0x07,// CAN
	0x3e, 0x2b, 0x04, 0x04, 0x0b,// EM
	0x19, 0x26, 0x0f, 0x0a, 0x05,// SUB
	0x3e, 0x2a, 0x04, 0x0a, 0x09,// ESC
	0x3e, 0x28, 0x04, 0x0a, 0x09,// FS
	0x1c, 0x22, 0x2e, 0x0a, 0x09,// GS
	0x3e, 0x28, 0x16, 0x0a, 0x09,// RS
	0x3c, 0x02, 0x02, 0x3e, 0x09,// US
	0x00, 0x00, 0x00, 0x00, 0x00,// (space)
	0x00, 0x00, 0x7d, 0x00, 0x00,// !
	0x00, 0x70, 0x00, 0x70, 0x00,// "
	0x14, 0x7f, 0x14, 0x7f, 0x14,// #
	0x12, 0x2a, 0x7f, 0x2a, 0x24,// $
	0x62, 0x64, 0x08, 0x13, 0x23,// %
	0x36, 0x49, 0x55, 0x22, 0x05,// &
	0x00, 0x50, 0x60, 0x00, 0x00,// '
	0x00, 0x1c, 0x22, 0x41, 0x00,// (
	0x00, 0x41, 0x22, 0x1c, 0x00,// )
	0x08, 0x2a, 0x1c, 0x2a, 0x08,// *
	0x08, 0x08, 0x3e, 0x08, 0x08,// +
	0x00, 0x05, 0x06, 0x00, 0x00,// ,
	0x08, 0x08, 0x08, 0x08, 0x08,// -
	0x00, 0x03, 0x03, 0x00, 0x00,// .
	0x02, 0x04, 0x08, 0x10, 0x20,// /
	0x3e, 0x45, 0x49, 0x51, 0x3e,// 0
	0x00, 0x21, 0x7f, 0x01, 0x00,// 1
	0x21, 0x43, 0x45, 0x49, 0x31,// 2
	0x42, 0x41, 0x51, 0x69, 0x46,// 3
	0x0c, 0x14, 0x24, 0x7f, 0x04,// 4
	0x72, 0x51, 0x51, 0x51, 0x4e,// 5
	0x1e, 0x29, 0x49, 0x49, 0x06,// 6
	0x40, 0x47, 0x48, 0x50, 0x60,// 7
	0x36, 0x49, 0x49, 0x49, 0x36,// 8
	0x30, 0x49, 0x49, 0x4a, 0x3c,// 9
	0x00, 0x36, 0x36, 0x00, 0x00,// :
	0x00, 0x35, 0x36, 0x00, 0x00,// ;
	0x00, 0x08, 0x14, 0x22, 0x41,// <
	0x14, 0x14, 0x14, 0x14, 0x14,// =
	0x41, 0x22, 0x14, 0x08, 0x00,// >
	0x20, 0x40, 0x45, 0x48, 0x30,// ?
	0x26, 0x49, 0x4f, 0x41, 0x3e,// @
	0x3f, 0x44, 0x44, 0x44, 0x3f,// A
	0x7f, 0x49, 0x49, 0x49, 0x36,// B
	0x3e, 0x41, 0x41, 0x41, 0x22,// C
	0x7f, 0x41, 0x41, 0x22, 0x1c,// D
	0x7f, 0x49, 0x49, 0x49, 0x41,// E
	0x7f, 0x48, 0x48, 0x40, 0x40,// F
	0x3e, 0x41, 0x41, 0x45, 0x26,// G
	0x7f, 0x08, 0x08, 0x08, 0x7f,// H
	0x00, 0x41, 0x7f, 0x41, 0x00,// I
	0x02, 0x01, 0x41, 0x7e, 0x40,// J
	0x7f, 0x08, 0x14, 0x22, 0x41,// K
	0x7f, 0x01, 0x01, 0x01, 0x01,// L
	0x7f, 0x20, 0x10, 0x20, 0x7f,// M
	0x7f, 0x10, 0x08, 0x04, 0x7f,// N
	0x3e, 0x41, 0x41, 0x41, 0x3e,// O
	0x7f, 0x48, 0x48, 0x48, 0x30,// P
	0x3e, 0x41, 0x45, 0x42, 0x3d,// Q
	0x7f, 0x48, 0x4c, 0x4a, 0x31,// R
	0x31, 0x49, 0x49, 0x49, 0x46,// S
	0x40, 0x40, 0x7f, 0x40, 0x40,// T
	0x7e, 0x01, 0x01, 0x01, 0x7e,// U
	0x7c, 0x02, 0x01, 0x02, 0x7c,// V
	0x7f, 0x02, 0x0c, 0x02, 0x7f,// W
	0x63, 0x14, 0x08, 0x14, 0x63,// X
	0x60, 0x10, 0x0f, 0x10, 0x60,// Y
	0x43, 0x45, 0x49, 0x51, 0x61,// Z
	0x00, 0x00, 0x7f, 0x41, 0x41,// [
	0x20, 0x10, 0x08, 0x04, 0x02,// "\"
	0x41, 0x41, 0x7f, 0x00, 0x00,// ]
	0x10, 0x20, 0x40, 0x20, 0x10,// ^
	0x01, 0x01, 0x01, 0x01, 0x01,// _
	0x00, 0x40, 0x20, 0x10, 0x00,// `
	0x02, 0x15, 0x15, 0x15, 0x0f,// a
	0x7f, 0x09, 0x11, 0x11, 0x0e,// b
	0x0e, 0x11, 0x11, 0x11, 0x02,// c
	0x0e, 0x11, 0x11, 0x09, 0x7f,// d
	0x0e, 0x15, 0x15, 0x15, 0x0c,// e
	0x08, 0x3f, 0x48, 0x40, 0x20,// f
	0x08, 0x14, 0x15, 0x15, 0x1e,// g
	0x7f, 0x08, 0x10, 0x10, 0x0f,// h
	0x00, 0x11, 0x5f, 0x01, 0x00,// i
	0x02, 0x01, 0x11, 0x5e, 0x00,// j
	0x00, 0x7f, 0x04, 0x0a, 0x11,// k
	0x00, 0x41, 0x7f, 0x01, 0x00,// l
	0x1f, 0x10, 0x0c, 0x10, 0x0f,// m
	0x1f, 0x08, 0x10, 0x10, 0x0f,// n
	0x0e, 0x11, 0x11, 0x11, 0x0e,// o
	0x1f, 0x14, 0x14, 0x14, 0x08,// p
	0x08, 0x14, 0x14, 0x0c, 0x1f,// q
	0x1f, 0x08, 0x10, 0x10, 0x08,// r
	0x09, 0x15, 0x15, 0x15, 0x02,// s
	0x10, 0x7e, 0x11, 0x01, 0x02,// t
	0x1e, 0x01, 0x01, 0x02, 0x1f,// u
	0x1c, 0x02, 0x01, 0x02, 0x1c,// v
	0x1e, 0x01, 0x06, 0x01, 0x1e,// w
	0x11, 0x0a, 0x04, 0x0a, 0x11,// x
	0x18, 0x05, 0x05, 0x05, 0x1e,// y
	0x11, 0x13, 0x15, 0x19, 0x11,// z
	0x00, 0x08, 0x36, 0x41, 0x00,// {
	0x00, 0x00, 0x7f, 0x00, 0x00,// |
	0x00, 0x41, 0x36, 0x08, 0x00,// }
	0x08, 0x08, 0x2a, 0x1c, 0x08,// ->
	0x08, 0x1c, 0x2a, 0x08, 0x08,// <-
};

#endif /* FONT_H */
