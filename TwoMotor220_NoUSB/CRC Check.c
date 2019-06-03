/* 	CRC Check.c 
 *
 *	This file contains the CRCcalculate()
 *	for doing CRC's for Modbus communications.
 *	This CRC is needed for Modbus serial packets
 *	but not for TCP/IP since Ethernet protocol 
 *	has its own built-in CRC or something similiar.
 *
 *  Modbus CRC's are two byte integers
 *  and require the table crc_tab16[] declared below.
 *
 *	The routines in this file were found at:
 *	http://www.lammertbies.nl/comm/info/crc-calculation.html
 *
 *	File download "lib_crc.zip" at
 *	http://www.lammertbies.nl/comm/software/index.html
 *
 *  12-29-16: Added getCRC7() and reverseByte()
 *  Added parity to create getCRC8()
 */
 
#define	MODBUS_POLYNOMIAL	0xA001

const unsigned short crc_tab16[256]={
0x0,0xC0C1,0xC181,0x140,0xC301,0x3C0,0x280,0xC241,0xC601,0x6C0,0x780,0xC741,0x500,0xC5C1,0xC481,0x440,
0xCC01,0xCC0,0xD80,0xCD41,0xF00,0xCFC1,0xCE81,0xE40,0xA00,0xCAC1,0xCB81,0xB40,0xC901,0x9C0,0x880,0xC841,
0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

unsigned char testBuffer[128];
unsigned char reverseByte(unsigned char dataByte);

/*******************************************************************\
*                                                                   *
*   static void init_crc16_tab( void );                             *
*                                                                   *
*   The function init_crc16_tab() was used to fill      			*
*	the above array crc_tab16[]										*
*   for update_crc_16 with the values shown.                        *
*																	*
*                                                                   *
\*******************************************************************/
/*
static void init_crc16_tab	(void) {
unsigned short i, j, k, crc, c;

	k=0;
	putsUART2("\r\r");
	for (i=0; i<256; i++) {
        crc=0;
        c=(unsigned short) i;
        for (j=0; j<8; j++){
            if ((crc^c)&0x0001) 
            	crc=(crc>>1)^MODBUS_POLYNOMIAL;
            else
            	crc=crc>>1;
            c=c>>1;
        }
        crc_tab16[i] = crc;
        sprintf (testBuffer, "0x%X,", crc);
		putsUART2(testBuffer);

		k++;
		if(k==16){
			putsUART2("\r");
			k=0;
		}			
    }
}  
*/


/*******************************************************************\
*                                                                   *
*   unsigned short update_crc_16( unsigned short crc, char c );     *
*                                                                   *
*   The function update_crc_16 calculates a  new  CRC-16  value     *
*   based  on  the  previous value of the CRC and the next byte     *
*   of the data to be checked.                                      *
*                                                                   *
\*******************************************************************/

unsigned short update_crc_16 (unsigned short crc, unsigned char nextVal) {
unsigned short tmp, shortVal;

	shortVal = 0x00ff & (unsigned short) nextVal; 
    tmp =  crc^shortVal;
    crc = (crc >> 8) ^ crc_tab16 [tmp&0xff];
    return crc;
} 

//	CRCcalculate():
//	This routine does the complete CRC for Modbus serial communication.
// 	The input message pointer accepts an array of bytes
// 	to be used for calculating a CRC for Modbus.
// 	nBytes is the number of bytes to be used for the CRC.
//	It calls the above subroutine update_crc_16()
//	for each byte, and returns resulting CRC.
unsigned short CRCcalculate (unsigned char *message, unsigned char nBytes){
unsigned short CRCresult=0xFFFF;
unsigned char i;
	
	//init_crc16_tab();	This does not need to be called if the array is initialized in ROM above.
	
	CRCresult=0xFFFF; // For Modbus CRC, use 0xFFFF as the seed value

	for (i=0; i<nBytes; i++)	
		CRCresult=update_crc_16	(CRCresult, message[i]);
	
	return(CRCresult);
}	


// This flips the order of bits in the input data byte
// and returns the result, so the LSB is first and MSB is last
unsigned char reverseByte(unsigned char dataByte){
    unsigned char i, byteReversed, byteMask, reverseMask;
        
    byteReversed = 0x00;
    byteMask = 0x01;
    reverseMask = 0x80;
    for (i = 0; i < 8; i++) {
        if (dataByte & byteMask) byteReversed |= reverseMask;
        byteMask = byteMask << 1;
        reverseMask = reverseMask >> 1;
    }   
    return (byteReversed);
}

#define NULL 0
#define CRCpolynomlial 0b10001001  // This is 0x91 flipped
// This routine generates a seven bit CRC 
// using data from an array of one or more bytes.
// The CRC is reversed to create the return value
// with an MSB that is a 0.
unsigned char getCRC7(unsigned char *ptrMessage, short numBytes){
    unsigned char CRCresult, CRCbyte, byteMask, nextByte;    
    short i;

    if (ptrMessage == NULL || numBytes == 0) return(0);
    
    // Get the first message byte and spin it around, LSB first:
    CRCbyte = reverseByte(ptrMessage[0]); 
    
    // Get the next message byte. Bits will be fetched from it LSB first:
    if (numBytes > 1) nextByte = ptrMessage[1]; 
    else nextByte = 0x00;
    
    byteMask = 0x01;       
    i = 0;
    while (i < numBytes){
        // If the MSB is a '1', XOR byte, otherwise, do nothing:
        if (CRCbyte & 0x80) CRCbyte = CRCbyte ^ CRCpolynomlial;
        // Shift result up one bit, and set the LSB to 0:
        CRCbyte = (CRCbyte << 1) & 0xFE;
        // Get next message bit and OR it to LSB:
        if (nextByte & byteMask) CRCbyte |= 0x01;           
        // Shift mask up for next message bit:
        if (byteMask < 0x80) byteMask = byteMask << 1;
        // If all eight bits have been fetched,
        // get next message byte and reset mask:       
        else {            
            if (i < numBytes - 2) nextByte = ptrMessage[i + 2];
            else nextByte = 0x00;
            i++;
            byteMask = 0x01;
        }
    }
    // When all done, flip CRC around to get result:
    CRCresult = reverseByte(CRCbyte);
    return (CRCresult);
}
	

#define NULL 0
#define CRCpolynomlial 0b10001001  // This is 0x91 flipped
// This is the same as the above getCRC7, but with a parity bit added to the MSB
unsigned char getCRC8(unsigned char *ptrMessage, short numBytes){
    unsigned char CRCresult, CRCbyte, byteMask, nextByte, parity; 
    short i;

    if (ptrMessage == NULL || numBytes == 0) return(0);
    
    // Get the first message byte and spin it around, LSB first:
    CRCbyte = reverseByte(ptrMessage[0]); 
    
    // Get the next message byte. Bits will be fetched from it LSB first:
    if (numBytes > 1) nextByte = ptrMessage[1]; 
    else nextByte = 0x00;
    
    byteMask = 0x01;       
    i = 0;
    while (i < numBytes){
        // If the MSB is a '1', XOR byte, otherwise, do nothing:
        if (CRCbyte & 0x80) CRCbyte = CRCbyte ^ CRCpolynomlial;
        // Shift result up one bit, and set the LSB to 0:
        CRCbyte = (CRCbyte << 1) & 0xFE;
        // Get next message bit and OR it to LSB:
        if (nextByte & byteMask) CRCbyte |= 0x01;           
        // Shift mask up for next message bit:
        if (byteMask < 0x80) byteMask = byteMask << 1;
        // If all eight bits have been fetched,
        // get next message byte and reset mask:       
        else {            
            if (i < numBytes - 2) nextByte = ptrMessage[i + 2];
            else nextByte = 0x00;
            i++;
            byteMask = 0x01;
        }
    }
    // When all done, flip CRC around to get result:
    CRCresult = reverseByte(CRCbyte);
    
    parity = 0x00;
    for (i = 0; i < 4; i++) parity = parity + ptrMessage[i];
    if (parity & 0x01) CRCresult |= 0x80;    
    
    return (CRCresult);
}

		
