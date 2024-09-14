#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include "SI570.h"

// some constants
#define POW_2_28          268435456.0         // double precision floating point
                                              // calculations
#define SI570_RST   (1<<7)        // Reg 135 Reset bit
#define SI570_NewGreq   (1<<6)        // Reg 135 New Frequency setup bit
#define SI570_FreezeM   (1<<5)        // Reg 135 Freeze M bit
#define SI570_FreezeVCADC (1<<4)        // Reg 135 Freeze VCADC bit
#define SI570_RECALL    (1<<0)        // Reg 135 Factory config recall bit

#define SI570_FreezeDCO   (1<<4)        // Reg 137 Freeze DCO for new setup
#define SI570_ADDRESS           0x55

// Enter in the Startup frequency. This should be known when ordering the chip, but if not,
// this can be measured. This must be accurate to ensure accuracy of all other frequencies.
#define  FOUT_START_UP    56.32  // MHz

// Si570 FDCO Range
// These values should not be modified, but can be depending on application.
float FDCO_MAX = 5670;      //MHz
float FDCO_MIN = 4850;      //MHz

// These are the only valie values for HS_DIV. See datasheet for more information
unsigned char HS_DIV[6] = {11, 9, 7, 6, 5, 4};


//-----------------------------------------------------------------------------
// Module variables
//-----------------------------------------------------------------------------
static unsigned char REG[6];            // Array of bits that holds the initial values read from the Si57x

//static unsigned long INITIAL_RFREQ_LONG;// RFREQ value read from the Si57x when it when it is turned on

static unsigned long rfreq_int;   // calculation variables
static unsigned long rfreq_frac;
static long double RFREQ;         // Fractional multiplier used to achieve correct output frequency
static long double OLD_RFREQ = 0.0;   // Old value of Fractional multiplier used for soft tune

static float FXTAL = 114.285;     // Will hold the value of the internal crystal frequency

static unsigned long FRAC_BITS;         // Double Floating Point Precision Method

static unsigned char n1_now = 0;        // Output divider that is modified and used in calculating the new RFREQ
static unsigned char old_n1 = 0;  // old value used for soft tune

static unsigned char hsdiv_now = 0;     // Output divider that is modified and used in calculating the new RFREQ
static unsigned char old_hsdiv = 0; // old value used for soft tune

static float old_freq = 0.0;    // last frequency setup (MHz)

//-----------------------------------------------------------------------------
// Helper functions
//-----------------------------------------------------------------------------

// This function sets appropriate bits in an unsigned char variable

unsigned char SetBits(unsigned char original, unsigned char reset_mask, unsigned char new_val) {
    return ((original & reset_mask) | new_val);
}


// check if large frequency change happened

int check_if_large_change(void) {
    long double delta_rfreq;

    if (hsdiv_now != old_hsdiv) {
        return 1; // major config change
    }

    if (n1_now != old_n1) {
        return 1; // major config change
    }

    if (OLD_RFREQ == RFREQ) {// nothing changed
        return 0;
    }

    if (OLD_RFREQ < RFREQ) {
        delta_rfreq = RFREQ - OLD_RFREQ; // Old smaller
    } else {
        delta_rfreq = OLD_RFREQ - RFREQ; // Old larger
    }

    if ((delta_rfreq / OLD_RFREQ) < 0.00345) { // check the 3500ppm limit (keep it well in the limits)
        return 0; // small change
    } else {
        return 1; // large change
    }
}

void writeReg8(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(SI570_ADDRESS);
  Wire.write(addr);            
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg8(uint8_t addr) {
  Wire.beginTransmission(SI570_ADDRESS); 
  Wire.write(addr); 
  Wire.endTransmission(0);
   
  Wire.requestFrom(SI570_ADDRESS, 1); 
  return Wire.read();
}
//-----------------------------------------------------------------------------
// ReadStartUpConfig
//-----------------------------------------------------------------------------
//
// Reads start-up register contents for RFREQ, HS_DIV, and N1 
// and calculates the internal crystal frequency (FXTAL)

void si570_read_config(void) {
    int i;
    Wire.begin();
    // Writes 0x01 to register 135. This will recall NVM bits into RAM. 
    // See register 135 in Si57x datasheet for more information
    writeReg8(135, SI570_RECALL);
    delay(1);
    // read registers 7 to 12 of Si57x REG[0] is equivalent to register 7 in the 
    // device, REG[5] is register 12 in the device
    for (i = 0; i < 6; i++) {
        REG[i] = readReg8(i + 7);
    }
    
    
    // Get value of INITIAL_HSDIV from REG[0] 4 is added to this 
    // because the bits "000" correspond to an HSDIV of 4
    hsdiv_now = ((REG[0] & 0xE0) >> 5) + 4;
    // Get correct value of N1 by adding parts of REG[0] and REG[1]
    n1_now = ((REG[0] & 0x1F) << 2) + ((REG[1] & 0xC0) >> 6);
    if (n1_now == 0) {
        n1_now = 1; // This is a corner case of N1
    } else if ((n1_now & 1) != 0) {
        // As per datasheet, illegal odd divider values should should be 
        // rounded up to the nearest even value.
        n1_now = n1_now + 1; 
    }


    rfreq_int = (REG[1] & 0x3F);
    rfreq_int = (rfreq_int << 4) + ((REG[2] & 0xF0) >> 4);

    rfreq_frac = (REG[2] & 0x0F);
    rfreq_frac = (rfreq_frac << 8) + REG[3];
    rfreq_frac = (rfreq_frac << 8) + REG[4];
    rfreq_frac = (rfreq_frac << 8) + REG[5];

    RFREQ = rfreq_int + rfreq_frac / POW_2_28;


    // Crystal Frequency (FXTAL) calculation
    FXTAL = (FOUT_START_UP * n1_now * hsdiv_now) / RFREQ; // XTAL frequency = decimal MHz
}


//-----------------------------------------------------------------------------
// RunFreqProg
//-----------------------------------------------------------------------------
//
// Parameters   : new_freq, new frequency in MHz + decimals
//
// Program Si570 to output the new_freq
//

void si570_freq_prog(float new_freq) {
    // Temporary counter variable used in for loops
    unsigned char i;

    // Flag that is set to 1 if a valid combination of N1 and HS_DIV is found
    int validCombo;

    // Stores the contents of Register 137 on Si57x
    unsigned char reg137;
    // Stores the contents of Register 135 on Si57x
    unsigned char reg135;

    // Maximum divider for HS_DIV and N1 combination
    unsigned int divider_max;
    // Minimum divider for HS_DIV and N1 combination
    unsigned int curr_div;
    // Used in Double Floating Point Precision Method
    unsigned int whole;

    // Used to calculate the final N1 to send to the
    float curr_n1;
    // Si570
    float n1_tmp;

    // Output divider that is modified and used in calculating the new RFREQ
    unsigned char n1;
    // Output divider that is modified and used in calculating the new RFREQ
    unsigned char hsdiv;

    // Find dividers (get the max and min divider 
    //range for the HS_DIV and N1 combo)
    divider_max = floorf(FDCO_MAX / new_freq);
    curr_div = ceilf(FDCO_MIN / new_freq);
    validCombo = 0;

    while (curr_div <= divider_max) {
        //check all the HS_DIV values with the next curr_div
        for (i = 0; i < 6; i++) {
            // get the next possible n1 value
            hsdiv = HS_DIV[i];
            curr_n1 = (float) (curr_div) / (float) (hsdiv);

            // Determine if curr_n1 is an integer and an 
            // even number or one then it will be a valid 
            // divider option for the new frequency
            n1_tmp = floorf(curr_n1);
            n1_tmp = curr_n1 - n1_tmp;
            // Then curr_n1 is an integer
            if (n1_tmp == 0.0) {
                n1 = (unsigned char) curr_n1;
                // Then the calculated N1 is
                // added n1 check <= 128 M. Collins KF4BQ suggested fix
                if (((n1 == 1) || ((n1 & 1) == 0)) && (curr_n1 <= 128)) {
                    // either 1 or an even number
                    validCombo = 1;
                }

            }
            // Divider was found, exit loop
            if (validCombo == 1) {
                break;
            }
        }

        // Divider was found, exit loop
        if (validCombo == 1) {
            break;
        }
        // If a valid divider is not found, increment curr_div and loop
        curr_div = curr_div + 1;
    }
    // save values for later use
    n1_now = n1;
    hsdiv_now = hsdiv;

    // If validCombo == 0 at this point, then there is an error 
    // in the calculation. Check if the provided FOUT0 and FOUT1 
    // are valid frequencies

    // New RFREQ calculation
    RFREQ = ((long double) new_freq * (long double) (n1 * hsdiv)) / FXTAL;
    // Subtract 4 because of the offset of HS_DIV. 
    // Ex: "000" maps to 4, "001" maps to 5   
    hsdiv = hsdiv - 4;
    // set the top 3 bits of REG[0] which will correspond to Register 7 on Si57x   
    REG[0] = (hsdiv << 5);

    if (n1 == 1) {
        //Corner case for N1. If N1=1, it is represented as "00000000"
        n1 = 0;
    } else if ((n1 & 1) == 0) {
        // If n1 is even, round down to closest odd number. See the
        // Si57x datasheet for more information.
        n1 = n1 - 1;
    }
    // Write correct new values to REG[0] through REG[6]. These will be sent 
    // to the Si57x and will update the output frequency

    // Set N1 part of REG[0]
    REG[0] = SetBits(REG[0], 0xE0, (n1 >> 2));
    // Set N1 part of REG[1]
    REG[1] = (n1 & 3) << 6;

    // Convert new RFREQ to the binary 
    // representation separate the integer part
    whole = floorf(RFREQ);
    // get the binary representation of the fractional part   
    FRAC_BITS = floorf((RFREQ - whole) * POW_2_28);

    // set reg 12 to 10 making frac_bits smaller by shifting off the 
    // last 8 bits every time
    for (i = 5; i >= 3; i--) {
        REG[i] = FRAC_BITS & 0xFF;
        FRAC_BITS = FRAC_BITS >> 8;
    }
    // set the last 4 bits of the fractional portion in reg 9
    REG[2] = SetBits(REG[2], 0xF0, (FRAC_BITS & 0xF));
    // set the integer portion of RFREQ across reg 8 and 9
    REG[2] = SetBits(REG[2], 0x0F, (whole & 0xF) << 4);
    REG[1] = SetBits(REG[1], 0xC0, (whole >> 4) & 0x3F);

    // check change magnitude and program the chip
    //int i2c_result = -1;
    if (check_if_large_change()) {
        // Read the current state of Register 137
        reg137 = readReg8(137);
        // Set the Freeze DCO bit in that register 
        // This must be done in order to update Registers 7-12 on the Si57x
        writeReg8(137, reg137 | SI570_FreezeDCO);

        for (i = 0; i < 6; i++) {
            // Write the new values to Registers 7-12
            writeReg8(i + 7, REG[i]);
        }
        // Read the current state of Register 137
        reg137 = readReg8(137);
        // Clear the Freeze DCO bit
        writeReg8(137, reg137 & ~SI570_FreezeDCO);
        // set NewFreq bit only when large change happened
        reg135 = readReg8(135);
        // Set the NewFreq bit to alert the DPSLL that a new frequency 
        // configuration has been applied
        writeReg8(135, reg135 | SI570_NewGreq);
        // wait for action completed
        delay(1);
        // set current value to old
        OLD_RFREQ = RFREQ;
        old_n1 = n1_now;
        old_hsdiv = hsdiv_now;
    } else {
        reg135 = readReg8(135);
        // Freeze M while writing new RFREQ
        writeReg8(135, reg135 | SI570_FreezeM);

        for (i = 1; i < 6; i++) {
            // Write the new values to Registers 8-12
            writeReg8(i + 7, REG[i]);
        }

        // Unfreeze M
        reg135 = readReg8(135);
        writeReg8(135, reg135 & ~SI570_FreezeM);
    }

}
// calculate & set Si570 frequency

void si570_calc_set_freq(unsigned long freq) {
    long double d;

    //  d = 4 * freq;             
    // LO = 4 * F rxtx
    d = freq; // convert to float
    d = d / 1000000.0; // Si570 set value = decimal MHz
    old_freq = d; // conver to float

    si570_freq_prog(old_freq); // set Si570
}

// set new XTAL frequency

void si570_set_xtal(float fxtal) // set new xtal frequency
{
    FXTAL = fxtal; // store neq xtal value

    OLD_RFREQ = 0; // force large update during next setup
    if (old_freq > 10.0) // if last frequeccy setup(MHz) is in range 
        si570_freq_prog(old_freq); // update frequency setup
}

// get XTAL frequency

float si570_get_xtal(void) {
    return FXTAL;
}


SI570::SI570(bool _init) {
  if (_init) {
    init();  
  }
}

SI570::init() {
  si570_read_config();
}

SI570::set_frequency(unsigned long frequency) {
  si570_calc_set_freq(frequency);
}